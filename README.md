# 🚀 Super Resolution AI 가속기 최적화 과제

---

## 💡 최적화 개요

CNN 연산 과정에 사용되는 가중치(Weights), 편향(Biases), 스케일(Scales) 값과 입력 데이터(Input Data)의 저장 및 접근 방식을 최적화하여 CNN 가속기의 성능과 효율을 향상시키는 것을 목표로 합니다.

<br/>
<img width="380" height="375" alt="Image" src="https://github.com/user-attachments/assets/03425d2b-1511-4b8c-a242-54dc19177a6b" />

---
## ✅ 최적화 1: 가중치/편향 데이터의 제로-스키핑 (Zero-Skipping)

CNN 연산에 사용될 사전 계산된 가중치, 편향, 스케일 값들은 아래와 같은 `.hex` 파일 형식으로 저장되어 있습니다.

<img width="800" alt="Hex Files" src="https://github.com/user-attachments/assets/a69db5bb-2c13-4bd7-8fd4-21ebb74f31ca" />

버퍼의 크기를 효율적으로 사용하기 위해, 값이 **'0'인 데이터를 생략하고 0이 아닌 값만 저장**하는 제로-스키핑(Zero-Skipping) 최적화를 적용했습니다.

<img width="800" alt="Zero Skipping Method" src="https://github.com/user-attachments/assets/21da6bd4-d8b8-4fa2-90e9-c6df829658fe" />

### 변환 전후 비교
<table align="center">
 <tr>
  <td align="center">
   <b>변환 전 (Before)</b><br/>
   <img width="400" alt="Before" src="https://github.com/user-attachments/assets/d69b266d-1781-4f16-bc83-6aa58efcb632" />
  </td>
  <td align="center" style="vertical-align: middle;">
   <br/>
   <h3>➡️</h3>
  </td>
  <td align="center">
   <b>변환 후 (After)</b><br/>
   <img width="400" alt="After" src="https://github.com/user-attachments/assets/9b523713-9137-4db3-908e-148f4b275107" />
  </td>
 </tr>
</table>

---
## ✅ 최적화 2: 입력 데이터 리오더링 (Input Data Reordering)

입력 데이터로 사용되는 흑백(Grayscale) 이미지의 `.hex` 파일은 한 줄(32비트)당 하나의 픽셀 값(8비트)만 저장하여 **24비트의 공간이 낭비**되고 있었습니다.

<img width="800" alt="Data Waste" src="https://github.com/user-attachments/assets/f8742327-4cb8-4dc2-aae5-ff42078709bf" />

이러한 비효율을 개선하기 위해 **하나의 32비트 워드(word)에 4개의 픽셀(8비트 x 4)을 순서대로 담아** 공간 낭비를 없애는 데이터 리오더링을 적용했습니다.

<img width="800" alt="Data Reordering" src="https://github.com/user-attachments/assets/b5fcc739-5ba2-48d3-af3b-b3f5773e7c17" />

<br/>

---
## ✅ 최적화 3: 병렬 데이터 로드 (Parallel Data Loading)

데이터 리오더링을 통해 이제 순차적으로 한 픽셀씩 읽어오는 대신, **한 번의 메모리 접근으로 4개의 픽셀을 동시에 병렬적으로 읽어오는** 것이 가능해졌습니다. 이를 통해 데이터 로드 속도를 크게 향상시켰습니다.

<img width="800" alt="Parallel Loading" src="https://github.com/user-attachments/assets/b5fcc739-5ba2-48d3-af3b-b3f5773e7c17" />

# 🩺 Wearable Device 제작 프로젝트

<br/>

<table align="center">
 <tr>
  <td align="center">
   <b>시스템 구성도</b><br/><br/>
   <img width="500" alt="System Block Diagram" src="https://github.com/user-attachments/assets/c5d999e9-f7bf-4192-9b1d-90467a1df916" />
  </td>
  <td align="center">
   <b>주요 흐름</b><br/><br/>
   <img width="300" alt="Flow Chart" src="https://github.com/user-attachments/assets/39d275a6-e086-43a0-b70a-e6e8e464a40d" />
  </td>
 </tr>
</table>

<br/>

---
## 📂 소스 코드

자세한 사항은 `WearableDevice` 폴더에 전체 소스 코드가 있습니다.

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
