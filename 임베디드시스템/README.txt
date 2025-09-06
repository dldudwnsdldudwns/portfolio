driver 폴더 안에 디바이스 드라이버 파일 

test 폴더 안에 led + 7seg 프로그램 파일

emotion.onnx는 michellejieli/emotion_text_classifier 모델을 onnx로 convert 한 파일

in pi 폴더는 실제 라즈베리파이에 들어갈 폴더

sudo inmod led7seg.ko - > sudo ./predict.py (숫자 0~3)