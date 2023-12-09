# RobotExperiment4-TermProject

2023 2학기 로봇학실험 4 프로젝트 코드입니다.  

팀원 : [강기태], [윤원호](https://github.com/yun-goon)


|OS|Langauge|개발 환경|   
|:---:|:---:|:---:|   
|Window 11|C++(MFC), C(Atmega128)|Visual Studio 2017, Atmel Studio 7.0|   

## 프로젝트 설명  

One Arm Robot을 실제로 제작하고 위치, 속도, 전류 값을 제어하는 것을 목표로 합니다.  
ODE와 MFC를 활용하여 Simulation 환경을 구성하였고,  
Main PC와 Atmega128를 Serial 통신을 활용하여 Data를 주고받습니다.  
NTGraph를 활용하여 실제 Motor의 Data를 MFC상에서 시각화를 진행했습니다.  

제어기는 Cascade 구조로 설계하였으며  
위치 제어기는 PD 제어기, 속도 제어기는 PI 제어기, 전류 제어기는 PI 제어기로 설계했습니다.  
Anti windup을 추가하여 정상상태오차를 제거하였습니다.  



## 결과 영상

**Target Position을 90 degree, Target Velocity를 10, Target Torque를 0.1로 설정했을 때 결과**  


<p align="center"><img src="https://github.com/yun-goon/RobotExperiment4-TermProject/assets/92098771/a55d73dd-ba45-4135-907e-6b38458e269e" width="600px"></p>  
<p align="center"> < 실제 Motor 움직임 (x2) ></p>  


---
**Target Position을 90 degree, Target Velocity를 10deg/s, Target Torque를 0.05[Nm]로 설정하고 외란을 적용한 결과**  

<p align="center"><img src="https://github.com/yun-goon/RobotExperiment4-TermProject/assets/92098771/1b8c93c3-048d-4321-bc5f-749cce4fc4b9" width="600px"></p>  
<p align="center"> < 실제 Motor 움직임 (x2) ></p>  
