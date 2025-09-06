안녕하세요 해당 직무에 지원한 이영준입니다.
[제 포트폴리오 저장소 보러가기](https://github.com/dldudwnsdldudwns/portfolio)

***

##  Super Resolution AI 가속기 최적화 과제

###  최적화 개요

CNN 연산 과정에 사용되는 가중치(Weights), 편향(Biases), 스케일(Scales) 값과 입력 데이터(Input Data)의 저장 및 접근 방식을 최적화하여 CNN 가속기의 성능과 효율을 향상시키는 것을 목표로 합니다.

<br/>
<img width="380" height="375" alt="Image" src="https://github.com/user-attachments/assets/03425d2b-1511-4b8c-a242-54dc19177a6b" />

###  최적화 1: 가중치/편향 데이터의 제로-스키핑 (Zero-Skipping)

CNN 연산에 사용될 사전 계산된 가중치, 편향, 스케일 값들은 아래와 같은 `.hex` 파일 형식으로 저장되어 있습니다.

<img width="800" alt="Hex Files" src="https://github.com/user-attachments/assets/a69db5bb-2c13-4bd7-8fd4-21ebb74f31ca" />

버퍼의 크기를 효율적으로 사용하기 위해, 값이 **'0'인 데이터를 생략하고 0이 아닌 값만 저장**하는 제로-스키핑(Zero-Skipping) 최적화를 적용했습니다.

<img width="800" alt="Zero Skipping Method" src="https://github.com/user-attachments/assets/21da6bd4-d8b8-4fa2-90e9-c6df829658fe" />

#### 변환 전후 비교
<table align="center">
 <tr>
  <td align="center">
   <b>변환 전 (Before)</b><br/>
   <img width="400" alt="Before" src="https://github.com/user-attachments/assets/d69b266d-1781-4f16-bc83-6aa58efcb632" />
  </td>
  <td align="center" style="vertical-align: middle;">
   <br/>
   <h3> </h3>
  </td>
  <td align="center">
   <b>변환 후 (After)</b><br/>
   <img width="400" alt="After" src="https://github.com/user-attachments/assets/9b523713-9137-4db3-908e-148f4b275107" />
  </td>
 </tr>
</table>

###  최적화 2: 입력 데이터 리오더링 (Input Data Reordering)

입력 데이터로 사용되는 흑백(Grayscale) 이미지의 `.hex` 파일은 한 줄(32비트)당 하나의 픽셀 값(8비트)만 저장하여 **24비트의 공간이 낭비**되고 있었습니다.

<img width="800" alt="Data Waste" src="https://github.com/user-attachments/assets/f8742327-4cb8-4dc2-aae5-ff42078709bf" />

이러한 비효율을 개선하기 위해 **하나의 32비트 워드(word)에 4개의 픽셀(8비트 x 4)을 순서대로 담아** 공간 낭비를 없애는 데이터 리오더링을 적용했습니다.

<img width="800" alt="Data Reordering" src="https://github.com/user-attachments/assets/b5fcc739-5ba2-48d3-af3b-b3f5773e7c17" />

###  최적화 3: 병렬 데이터 로드 (Parallel Data Loading)

데이터 리오더링을 통해 이제 순차적으로 한 픽셀씩 읽어오는 대신, **한 번의 메모리 접근으로 4개의 pixel을 동시에 병렬적으로 읽어오는** 것이 가능해졌습니다. 이를 통해 데이터 로드 속도를 크게 향상시켰습니다.

<img width="800" alt="Parallel Loading" src="https://github.com/user-attachments/assets/b5fcc739-5ba2-48d3-af3b-b3f5773e7c17" />

***

##  Wearable Device 제작 프로젝트

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

자세한 사항은 WearableDevice 폴더에 Source 코드가 있습니다.

***

##  임베디드 시스템 프로젝트

> 7seg, LED 디바이스 드라이버 구현 -> onnx를 활용하여 Text 기반 감정을 판별하는 BERT모델을 경량화 -> 엣지 임베디드 시스템 구현

<br/>

####  동작 사진 &  회로 사진

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

***

##  특허 유니버시아드: AI 반도체 R&D 전략 수립

> ‘인공지능 발전에 따른 AI 반도체’를 주제로 특허 빅데이터를 분석하고, 미래 유망 기술에 대한 R&D 전략 및 특허 확보 방안을 수립하는 공모전에 참가했습니다.

<br/>

####  담당 역할
- **정량분석 및 데이터 분석 (PDF p.37 ~ p.75)**: 특허 동향 및 기술 트렌드 분석
- **공백기술 도출 및 아이디어 제안 (PDF p.204 ~ p.216)**: 기업별 특허 포트폴리오 분석을 통한 R&D 전략 제안

<br/>

####  1. 특허 정량분석 및 핵심 인사이트

특허 빅데이터를 분석하여 AI 반도체 기술의 전체적인 동향을 파악했습니다.

<table align="center">
 <tr>
  <td align="center">
   <img width="420" alt="분석 결과 1" src="https://github.com/user-attachments/assets/d187d1c1-830b-4413-8574-b65e05843207" />
  </td>
  <td align="center">
   <img width="420" alt="분석 결과 2" src="https://github.com/user-attachments/assets/dcce3bfd-118d-4cf0-a505-f4abae2127a5" />
  </td>
 </tr>
</table>

#####  주요 분석 결과
> 분석 결과, 특히 인간의 뇌를 모방한 **뉴로모픽(Neuromorphic) 반도체** 관련 특허 출원이 폭발적으로 증가하고 있음을 확인했으며, 이를 핵심 유망 기술로 선정했습니다.

<br/>

####  2. 기업별 공백기술 탐색

조사한 특허 데이터를 기반으로, 청구범위 분석을 통해 각 기업별 기술 포트폴리오의 강점과 약점을 파악하고 선점 가능한 **공백기술(White Space)** 영역을 탐색했습니다.

<table align="center">
 <tr>
  <td align="center">
   <img width="420" alt="공백기술 분석 1" src="https://github.com/user-attachments/assets/ef13cfa2-3add-4cb6-b9e7-91a396d3a287" />
  </td>
  <td align="center">
   <img width="420" alt="공백기술 분석 2" src="https://github.com/user-attachments/assets/a08129e9-4eea-4bf0-a894-118b1ccb7299" />
  </td>
 </tr>
  <tr>
  <td colspan="2" align="center">
   <img width="600" alt="공백기술 분석 3" src="https://github.com/user-attachments/assets/51a4f92a-1117-49e3-b063-1e8c495a79db" />
  </td>
 </tr>
</table>

<br/>

####  3. R&D 전략 아이디어 제안

분석 결과를 바탕으로, **삼성(Samsung)의 기존 뉴로모픽 특허에 탐색한 공백기술을 적용**하여 특허 포트폴리오를 강화하고 기술 경쟁력을 확보하는 R&D 아이디어를 제안했습니다.

<table align="center">
 <tr>
  <td align="center">
   <img width="420" alt="아이디어 제안 1" src="https://github.com/user-attachments/assets/0097efc5-269a-41cb-b578-9a1fe2818164" />
  </td>
  <td align="center">
   <img width="420" alt="아이디어 제안 2" src="https://github.com/user-attachments/assets/61dc4415-4ee6-414e-bc8a-9d57de71cf65" />
  </td>
 </tr>
 <tr>
  <td align="center">
   <img width="420" alt="아이디어 제안 3" src="https://github.com/user-attachments/assets/e98299d6-32dc-4b7c-b383-86ff8cb9154c" />
  </td>
  <td align="center">
   <img width="420" alt="아이디어 제안 4" src="https://github.com/user-attachments/assets/e4b5925f-955f-41d8-8ca3-f0981397f356" />
  </td>
 </tr>
  <tr>
  <td colspan="2" align="center">
   <img width="420" alt="아이디어 제안 5" src="https://github.com/user-attachments/assets/c4c44f73-a516-4b97-9e49-58974af8747e" />
  </td>
 </tr>
</table>

#####  최종 특허 아이디어 도출
> 위와 같은 분석 근거와 특허 내용을 종합하여, 최종적으로 다음과 같은 신규 특허 아이디어를 도출했습니다.

<p align="center">
  <img width="700" alt="최종 도출 내용" src="https://github.com/user-attachments/assets/fd03d23e-9a2a-43a8-9bc5-ec88da5882d5" />
</p>

<br/>

####  참고 자료

더 구체적인 분석 내용 및 제안 아이디어는 `특허 유니버시아드` 폴더 안의 PDF 파일을 통해 확인하실 수 있습니다.


네, 그럼요. '졸음운전 방지 시스템' 프로젝트를 깔끔하고 보기 좋은 README 형식으로 만들어 드릴게요. 요청하신 대로 이미지들은 좌우로 나란히 배치했습니다.

아래 코드를 복사해서 README.md 파일에 그대로 붙여넣으시면 됩니다.

Markdown

##  졸음운전 방지 시스템 (Drowsy Driving Prevention System)

> Real-time CNN 모델을 이용하여 운전자의 자세와 눈 감김을 감지하고, 졸음운전 여부를 판단하여 경고하는 시스템을 개발했습니다.

- **주요 기술**: `CNN`, `Real-time Object Detection`, `Data Preprocessing`, `Python`, `OpenCV`
- **담당 역할**: **데이터 수집 자동화 및 전처리**

<br/>

### 데이터 처리 과정
<table align="center">
 <tr>
  <td align="center"><img width="260" alt="Data Processing 1" src="https://github.com/user-attachments/assets/ea267421-11eb-4e96-862b-71b31de28ceb" /></td>
  네, 그럼요. '졸음운전 방지 시스템' 프로젝트를 깔끔하고 보기 좋은 README 형식으로 만들어 드릴게요. 요청하신 대로 이미지들은 좌우로 나란히 배치했습니다.

아래 코드를 복사해서 README.md 파일에 그대로 붙여넣으시면 됩니다.

Markdown

## 😴 졸음운전 방지 시스템 (Drowsy Driving Prevention System)

> Real-time CNN 모델을 이용하여 운전자의 자세와 눈 감김을 감지하고, 졸음운전 여부를 판단하여 경고하는 시스템을 개발했습니다.

- **주요 기술**: `CNN`, `Real-time Object Detection`, `Data Preprocessing`, `Python`, `OpenCV`
- **담당 역할**: **데이터 수집 자동화 및 전처리**

<br/>

### 데이터 처리 과정
<table align="center">
 <tr>
  <td align="center"><img width="260" alt="Data Processing 1" src="https://github.com/user-attachments/assets/ea267421-11eb-4e96-862b-71b31de28ceb" /></td>
  <img width="1600" height="900" alt="image" src="https://github.com/user-attachments/assets/fafa5692-7592-4943-8afe-530b909981f5" />

  <img width="1600" height="900" alt="image" src="https://github.com/user-attachments/assets/724d9780-0df8-4d1b-8502-72ae4af9bfb3" />

 </tr>
</table>

<br/>

### 시스템 시연
<table align="center">
 <tr>
  <td align="center"><img width="350" alt="Demonstration 1" src="https://github.com/user-attachments/assets/7241708a-4e43-4b19-a42d-c31612311c7f" /></td>
  <td align="center"><img width="350" alt="Demonstration 2" src="https://github.com/user-attachments/assets/37890204-8de5-43de-8f69-62b61516716b" /></td>
 </tr>
</table>

<br/>

### 📂 소스 코드
자세한 내용은 프로젝트 폴더 안의 소스 파일을 통해 확인하실 수 있습니다.
  <td align="center"><img width="260" alt="Data Processing 3" src="https://github.com/user-attachments/assets/cac5928e-16fa-420c-9695-574cf2158c4e" /></td>
 </tr>
</table>

<br/>

### 시스템 시연
<table align="center">
 <tr>
  <td align="center"><img width="350" alt="Demonstration 1" src="https://github.com/user-attachments/assets/7241708a-4e43-4b19-a42d-c31612311c7f" /></td>
  <td align="center"><img width="350" alt="Demonstration 2" src="https://github.com/user-attachments/assets/37890204-8de5-43de-8f69-62b61516716b" /></td>
 </tr>
</table>

<br/>

###  소스 코드
자세한 내용은 프로젝트 폴더 안의 소스 파일을 통해 확인하실 수 있습니다.





