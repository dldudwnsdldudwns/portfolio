driver 폴더 안에 디바이스 드라이버 파일 

test 폴더 안에 led + 7seg 프로그램 파일

emotion.onnx는 michellejieli/emotion_text_classifier 모델을 onnx로 convert 진행 해야 함!

in pi 폴더는 실제 라즈베리파이에 들어갈 폴더

터미널에서 -> sudo inmod led7seg.ko - > sudo ./predict.py (숫자 0~3)

동작1~4사진, 회로사진이 첨부되어있음
<p align="center">
  <img width="600" alt="Image 1" src="https://github.com/user-attachments/assets/35af3c57-1e1d-4100-8214-6bd298a02a3f" /><br/>
  <img width="600" alt="Image 2" src="https://github.com/user-attachments/assets/1853f423-5469-4cbf-a97e-d2492cd4b31a" /><br/>
  <img width="600" alt="Image 3" src="https://github.com/user-attachments/assets/6aa893b6-904f-4f23-ad96-93c3ac34cec3" /><br/>
  <img width="600" alt="Image 4" src="https://github.com/user-attachments/assets/ee94fe2a-f78a-47b5-9166-070b24e4d412" /><br/>
  <img width="600" alt="Image 5" src="https://github.com/user-attachments/assets/99eb3bbd-424a-485a-a446-bbd733ec1f42" /><br/>
</p>
