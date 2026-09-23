#!/usr/bin/env python3
import os
import pickle
import time
from pathlib import Path

import numpy as np

from openpilot.system.hardware import TICI

os.environ["DEV"] = "QCOM" if TICI else "CPU"

from tinygrad.tensor import Tensor

from cereal import messaging
from cereal.messaging import PubMaster, SubMaster
from msgq.visionipc import VisionBuf, VisionIpcClient, VisionStreamType
from openpilot.common.file_chunker import read_file_chunked
from openpilot.common.params import Params
from openpilot.common.realtime import config_realtime_process
from openpilot.common.swaglog import cloudlog
from openpilot.common.transformations.camera import _ar_ox_fisheye, _os_fisheye
from openpilot.common.transformations.model import dmonitoringmodel_intrinsics
from openpilot.selfdrive.modeld.helpers import get_tg_input_devices
from openpilot.selfdrive.modeld.parse_model_outputs import safe_exp, sigmoid
from openpilot.selfdrive.monitoring.policy import DRIVER_MONITOR_SETTINGS
from openpilot.starpilot.common.model_lab import load_model_lab_config
from openpilot.system.hardware.usb import chestnut_firmware_ready
from openpilot.system.camerad.cameras.nv12_info import get_nv12_info

PROCESS_NAME = "selfdrive.modeld.dmonitoringmodeld"
SEND_RAW_PRED = os.getenv("SEND_RAW_PRED")
MODELS_DIR = Path(__file__).parent / "models"
MODEL_PKL_PATH = MODELS_DIR / "dmonitoring_model_tinygrad.pkl"
METADATA_PATH = MODELS_DIR / "dmonitoring_model_metadata.pkl"
DEFAULT_DMONITORING_CORES = 7
MODEL_LAB_DMONITORING_CORES = [0, 1, 2, 3]

_DM = DRIVER_MONITOR_SETTINGS()


def get_attentive_packet(frame_id: int, calib: np.ndarray, wheel_on_right: bool):
  """A driverStateV2 describing a driver looking straight ahead with their eyes open.

  Published in place of the model's own output while DisableDriverMonitoring is set, so that
  everything below this point - the policy and its timers, selfdrived's events, controlsd's
  forceDecel, the face on screen - keeps running on self-consistent data instead of having
  its conclusions overridden one at a time in half a dozen places.

  The face sits at the centre of the frame, so both focal angles in
  policy.face_orientation_from_model are zero, and the orientation is chosen so that function
  returns exactly the natural offsets the policy compares against:

      pitch = pitch_model + 0 - rpy_calib[1]   ->  pitch_model = PITCH_NATURAL + calib[1]
      yaw   = -yaw_model  + 0 - rpy_calib[2]   ->  yaw_model   = -(YAW_NATURAL + calib[2])

  Standard deviations are zero, under _HI_STD_THRESHOLD so the pose reads as low-std and
  under _DCAM_UNCERTAIN_ALERT_THRESHOLD so the camera never reads as uncertain.

  wheelOnRightProb feeds policy.py's wheelpos_offsetter on every frame above
  _WHEELPOS_CALIB_MIN_SPEED (faceProb below is 1.0, so the gate always passes here), and its
  filtered mean is what dmonitoringd.py periodically persists back into IsRhdDetected/IsRHD.
  A constant 0.5 is not neutral to that learner - it is exactly its own decision threshold - so
  the filtered mean converges on it and forces wheel_on_right to a fixed False, silently
  rewriting the saved side from fabricated data. Publish the side already saved instead, so the
  learner is self-consistent and the periodic persist is a genuine no-op.
  """
  msg = messaging.new_message('driverStateV2', valid=True)
  ds = msg.driverStateV2
  ds.frameId = frame_id
  ds.modelExecutionTime = 0.0
  ds.gpuExecutionTime = 0.0
  ds.rawPredictions = b''
  ds.wheelOnRightProb = 1.0 if wheel_on_right else 0.0
  orientation = [float(_DM._PITCH_NATURAL_OFFSET + calib[1]),
                 float(-(_DM._YAW_NATURAL_OFFSET + calib[2])), 0.0]
  for side in (ds.leftDriverData, ds.rightDriverData):
    side.faceOrientation = orientation
    side.faceOrientationStd = [0.0, 0.0, 0.0]
    side.facePosition = [0.0, 0.0]
    side.facePositionStd = [0.0, 0.0]
    side.faceProb = 1.0
    side.leftEyeProb = 1.0
    side.rightEyeProb = 1.0
    side.leftBlinkProb = 0.0
    side.rightBlinkProb = 0.0
    side.sunglassesProb = 0.0
    side.phoneProb = 0.0
    side.sleepProb = 0.0
  return msg


def dmonitoring_cpu_cores(params: Params, chestnut_ready: bool) -> int | list[int]:
  if chestnut_ready and load_model_lab_config(params)["enabled"]:
    return MODEL_LAB_DMONITORING_CORES
  return DEFAULT_DMONITORING_CORES


class ModelState:
  def __init__(self, cam_w: int, cam_h: int):
    self.device = get_tg_input_devices(PROCESS_NAME, usbgpu=False)["DEV"]
    with open(METADATA_PATH, "rb") as metadata_file:
      metadata = pickle.load(metadata_file)
    self.input_shapes = metadata["input_shapes"]
    self.output_slices = metadata["output_slices"]

    self.numpy_inputs = {"calib": np.zeros(self.input_shapes["calib"], dtype=np.float32)}
    self.tensor_inputs = {
      key: Tensor(value, device="NPY").realize()
      for key, value in self.numpy_inputs.items()
    }
    self.warp_numpy_inputs = {"transform": np.zeros((3, 3), dtype=np.float32)}
    self.warp_inputs = {
      key: Tensor(value, device="NPY").realize()
      for key, value in self.warp_numpy_inputs.items()
    }
    self.frame_size = get_nv12_info(cam_w, cam_h)[3]
    self._blob_cache: dict[int, Tensor] = {}
    self.model_run = pickle.loads(read_file_chunked(str(MODEL_PKL_PATH)))
    with open(MODELS_DIR / f"dm_warp_{cam_w}x{cam_h}_tinygrad.pkl", "rb") as warp_file:
      self.image_warp = pickle.load(warp_file)

  def run(self, buf: VisionBuf, calib: np.ndarray, transform: np.ndarray) -> tuple[np.ndarray, float]:
    self.numpy_inputs["calib"][0, :] = calib
    start = time.perf_counter()

    ptr = np.frombuffer(buf.data, dtype=np.uint8).ctypes.data
    if ptr not in self._blob_cache:
      self._blob_cache[ptr] = Tensor.from_blob(
        ptr, (self.frame_size,), dtype="uint8", device=self.device,
      )

    self.warp_numpy_inputs["transform"][:] = transform
    self.tensor_inputs["input_img"] = self.image_warp(
      self._blob_cache[ptr], self.warp_inputs["transform"],
    )
    output = self.model_run(**self.tensor_inputs).numpy().flatten()
    return output, time.perf_counter() - start


def slice_outputs(model_outputs, output_slices):
  return {key: model_outputs[np.newaxis, value] for key, value in output_slices.items()}


def parse_model_output(model_output):
  parsed = {"wheel_on_right": sigmoid(model_output["wheel_on_right"])}
  for suffix in ("lhd", "rhd"):
    face_descs = model_output[f"face_descs_{suffix}"]
    parsed[f"face_descs_{suffix}"] = face_descs[:, :-6]
    parsed[f"face_descs_{suffix}_std"] = safe_exp(face_descs[:, -6:])
    for key in (
      "face_prob",
      "left_eye_prob",
      "right_eye_prob",
      "left_blink_prob",
      "right_blink_prob",
      "sunglasses_prob",
      "using_phone_prob",
    ):
      parsed[f"{key}_{suffix}"] = sigmoid(model_output[f"{key}_{suffix}"])
    sleep_key = f"sleep_prob_{suffix}"
    parsed[sleep_key] = (
      sigmoid(model_output[sleep_key])
      if sleep_key in model_output
      else np.zeros((1, 1), dtype=np.float32)
    )
  return parsed


def fill_driver_data(msg, model_output, suffix):
  msg.faceOrientation = model_output[f"face_descs_{suffix}"][0, :3].tolist()
  msg.faceOrientationStd = model_output[f"face_descs_{suffix}_std"][0, :3].tolist()
  msg.facePosition = model_output[f"face_descs_{suffix}"][0, 3:5].tolist()
  msg.facePositionStd = model_output[f"face_descs_{suffix}_std"][0, 3:5].tolist()
  msg.faceProb = model_output[f"face_prob_{suffix}"][0, 0].item()
  msg.leftEyeProb = model_output[f"left_eye_prob_{suffix}"][0, 0].item()
  msg.rightEyeProb = model_output[f"right_eye_prob_{suffix}"][0, 0].item()
  msg.leftBlinkProb = model_output[f"left_blink_prob_{suffix}"][0, 0].item()
  msg.rightBlinkProb = model_output[f"right_blink_prob_{suffix}"][0, 0].item()
  msg.sunglassesProb = model_output[f"sunglasses_prob_{suffix}"][0, 0].item()
  msg.phoneProb = model_output[f"using_phone_prob_{suffix}"][0, 0].item()
  msg.sleepProb = model_output[f"sleep_prob_{suffix}"][0, 0].item()


def get_driverstate_packet(model_output, frame_id: int, exec_time: float, gpu_exec_time: float):
  msg = messaging.new_message("driverStateV2", valid=True)
  state = msg.driverStateV2
  state.frameId = frame_id
  state.modelExecutionTime = exec_time
  state.gpuExecutionTime = gpu_exec_time
  state.rawPredictions = model_output["raw_pred"]
  state.wheelOnRightProb = model_output["wheel_on_right"][0, 0].item()
  fill_driver_data(state.leftDriverData, model_output, "lhd")
  fill_driver_data(state.rightDriverData, model_output, "rhd")
  return msg


def run_frame(dm_disabled: bool, *, model: ModelState, pm: PubMaster, frame_id: int, calib: np.ndarray,
              wheel_on_right_saved: bool, buf: VisionBuf, model_transform: np.ndarray) -> None:
  """The bypass gate: main()'s loop calls this once per frame with the current dm_disabled
  value. When it's set, publish the synthetic attentive packet and skip the model entirely -
  the tinygrad forward pass never runs while the bypass is active, it isn't merely
  computed-then-discarded. Otherwise run the model and publish its real output, unchanged from
  before the bypass existed."""
  if dm_disabled:
    pm.send("driverStateV2", get_attentive_packet(frame_id, calib, wheel_on_right_saved))
    return

  start = time.perf_counter()
  model_output, gpu_execution_time = model.run(buf, calib, model_transform)
  execution_time = time.perf_counter() - start
  raw_pred = model_output.tobytes() if SEND_RAW_PRED else b""
  parsed = parse_model_output(slice_outputs(model_output, model.output_slices))
  parsed["raw_pred"] = raw_pred
  pm.send(
    "driverStateV2",
    get_driverstate_packet(parsed, frame_id, execution_time, gpu_execution_time),
  )


def main():
  params = Params()
  cpu_cores = dmonitoring_cpu_cores(params, chestnut_firmware_ready())
  config_realtime_process(cpu_cores, 5)
  cloudlog.info(f"driver monitoring CPU affinity: {cpu_cores}")
  cloudlog.warning("connecting to driver stream")
  vipc_client = VisionIpcClient("camerad", VisionStreamType.VISION_STREAM_DRIVER, True)
  while not vipc_client.connect(False):
    time.sleep(0.1)
  assert vipc_client.is_connected()
  cloudlog.warning(f"connected with buffer size: {vipc_client.buffer_len}")

  # connect() finishes before stream is populated. The first buffer always has known dimensions.
  first_buf = vipc_client.recv()
  while first_buf is None:
    first_buf = vipc_client.recv()

  model = ModelState(first_buf.width, first_buf.height)
  cloudlog.warning("models loaded, dmonitoringmodeld starting")

  sm = SubMaster(["liveCalibration"])
  pm = PubMaster(["driverStateV2"])
  params = Params()
  dm_disabled = params.get_bool("DisableDriverMonitoring")
  # Read once at startup to ensure a value before the first re-read; wheel_on_right_saved is
  # dmonitoringd.py's own last-saved side, persisted once per 6000 frames. Re-read periodically
  # so the cached value stays in sync with what dmonitoringd.py has persisted.
  wheel_on_right_saved = params.get_bool("IsRhdDetected")
  calib = np.zeros(model.numpy_inputs["calib"].size, dtype=np.float32)
  model_transform = None

  while True:
    buf = vipc_client.recv()
    if buf is None:
      continue

    if model_transform is None:
      camera = _os_fisheye if buf.width == _os_fisheye.width else _ar_ox_fisheye
      model_transform = np.linalg.inv(
        np.dot(dmonitoringmodel_intrinsics, np.linalg.inv(camera.intrinsics)),
      ).astype(np.float32)

    sm.update(0)
    if sm.updated["liveCalibration"]:
      calib[:] = np.array(sm["liveCalibration"].rpyCalib)

    # Re-read at 0.5 Hz so the switches take effect without a restart. The frame above is
    # still received either way, so camerad's stream is drained and the publish rate stays
    # at the camera's 20 Hz.
    if vipc_client.frame_id % 40 == 1:
      dm_disabled = params.get_bool("DisableDriverMonitoring")
      wheel_on_right_saved = params.get_bool("IsRhdDetected")
    run_frame(dm_disabled, model=model, pm=pm, frame_id=vipc_client.frame_id, calib=calib,
              wheel_on_right_saved=wheel_on_right_saved, buf=buf, model_transform=model_transform)


if __name__ == "__main__":
  try:
    main()
  except KeyboardInterrupt:
    cloudlog.warning("got SIGINT")
