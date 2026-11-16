WeChat QR code CNN detector + super-resolution models for OpenCV's `cv2.wechat_qrcode_WeChatQRCode`
(used by `scripts/shelf_scanner.py`), from https://github.com/WeChatCV/opencv_3rdparty (branch wechat_qrcode),
Apache-2.0. Without them the decoder takes the whole input as one candidate: it misses small codes in a large
frame and returns the input's corners instead of the code's (no position estimate possible).
