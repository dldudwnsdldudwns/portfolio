Super Resolution AI 가속기 최적화 과제

CNN 과정에서 이미 계산된 Weights , biases, scales 들이 저장된 hex 파일들은 다음과 같다.
<img width="1600" height="900" alt="Image" src="https://github.com/user-attachments/assets/a69db5bb-2c13-4bd7-8fd4-21ebb74f31ca" />

그래서 CNN 과정에서 사용되는 버퍼의 크기를 줄이기 위해 '0' 인 값을 생략하는 최적화 방법을 사용하였다.

<img width="1600" height="900" alt="Image" src="https://github.com/user-attachments/assets/2c7448f8-f6ea-458e-840c-2df73573805e" />

<table align="center">
 <tr>
  <td align="center">
   <b>변환 전 (Before)</b><br/>
   <img width="400" alt="변환 전 이미지" src="https://github.com/user-attachments/assets/d69b266d-1781-4f16-bc83-6aa58efcb632" />
  </td>
  <td align="center" style="vertical-align: middle;">
   <br/>
   <h3>➡️</h3>
  </td>
  <td align="center">
   <b>변환 후 (After)</b><br/>
   <img width="400" alt="변환 후 이미지" src="https://github.com/user-attachments/assets/9b523713-9137-4db3-908e-148f4b275107" />
  </td>
 </tr>
</table>





# 💻 임베디드 시스템 프로젝트

> 7seg, LED 디바이스 드라이버 구현 -> onnx를 활용하여 Text 기반 감정을 판별하는 BERT모델을 경량화 -> 엣지 임베디드 시스템 구현

<br/>

### 📸 동작 사진 & 🔌 회로 사진

<table align="center">
 <tr>
  <td align="center"><img width="300" alt="Image 1" src="https://github.com/user-attachments/assets/35af3c57-1e1d-4100-8214-6bd298a02a3f" /></td>
  <td align="center"><img width="300" alt="Image 2" src="https://github.com/user-attachments/assets/1853f423-5469-4cbf-a97e-d2492cd4b31a" /></td>
  <td align="center"><img width="300" alt="Image 3" src="https://github.com/user-attachments/assets/6aa893b6-904f-4f23-ad96-93c3ac34cec3" /></td>
 </tr>
 <tr>
  <td colspan="3" align="center">
   <table align="center">
    <tr>
     <td align="center"><img width="300" alt="Image 4" src="https://github.com/user-attachments/assets/ee94fe2a-f78a-47b5-9166-070b24e4d412" /></td>
     <td align="center"><img width="300" alt="Image 5" src="https://github.com/user-attachments/assets/99eb3bbd-424a-485a-a446-bbd733ec1f42" /></td>
    </tr>
   </table>
  </td>
 </tr>
</table>
