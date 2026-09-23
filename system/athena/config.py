# athenad holds a websocket open to comma and answers uploadFileToUrl / uploadFilesToUrls
# with any path on the device: a second route off this machine that the uploader's own
# block does not cover. These logs carry the owner's VIN, route history and raw camera
# footage, and this fork is not part of comma's fleet. Device access is by direct ssh.
#
# system/loggerd/uploader.py's own upload block (do_upload's non-fake_upload branch) does
# not read this constant -- it is a hardcoded, independent rewrite. Flipping ATHENA_ENABLED
# back to True restores athenad and the sidebar's CONNECT status but does not, by itself,
# restore uploads.
ATHENA_ENABLED = False
