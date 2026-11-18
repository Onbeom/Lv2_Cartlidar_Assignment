# 실제 카메라3개 보정 · 2026-10-29

## 현재 상태

사용자가 앞/뒤 RealSense2대와 Arducam1대를 Jetson에 USB 연결하고 cart11@cart11.local SSH 로그인 완료. 카메라는 아직 로봇 장착 전. 내부 보정/공장 파라미터 수집은 가능, 로봇 기준 외부 보정은 실제 장착 후 수행. 실제 보정 촬영·계산은 아직 안 했다.

## 보정판

모니터 표시용 ChArUco, DICT_5X5_100, 가로10×세로7칸, 마커35개, 내부코너54개. 명목 칸25mm·마커18mm(비율0.72). PDF는A4가로의실제치수지만모니터표시는실제물리치수와달라질수있다. 모니터의체커영역가로10칸전체폭/10과세로7칸전체높이/7을측정하고배율을고정한다. actual_display_square_mm는아직미측정. 내부파라미터는기하적축척과구분하며미터단위외부자세는실제크기필수.

- PDF: output/pdf/charuco_monitor_10x7_A4.pdf
- 순수패턴: charuco_10x7_5x5_100.png
- 사양: board_spec.json (실제표시크기 null은미확정)
- PDF렌더 검출확인: board_validation.json,35/35마커·54/54코너
- 도구: 261029_camera_calibration/make_board_pdf.py

일반흰색화면바탕으로표시, 가로세로비유지, 반사·화면줄무늬·흐림은촬영화면에서확인. 초점/촬영해상도를운용조건으로고정하고카메라를중앙/가장자리·기울기·거리변화로움직여관측한다. 모니터는RGB기하보정대상이며RealSense깊이보정을임의로덮어쓰지않는다.

## SSH 연결

공유앱터미널에서사용자비밀번호로그인확인. 에이전트별도BatchMode SSH는Permission denied(publickey,password)라원격점검을못했다. 명령실패를Jetson카메라오류로해석하지않는다. 비밀번호를채팅에요청/저장하지않는다.

PC새터미널에서아래명령으로연결을열고사용자가비밀번호를입력하면동일사용자제어소켓을통해후속자동점검할수있다(선택·권장,15분persist). 연결자체는카메라설정/팀파일을변경하지않는다.

```bash
ssh -o ControlMaster=auto -o ControlPersist=15m -o ControlPath=/tmp/lv2_camera_ssh.sock cart11@cart11.local
```

필요가끝나면PC에서`ssh -S /tmp/lv2_camera_ssh.sock -O exit cart11@cart11.local`로공유연결을종료할수있다. 비밀번호/sudo입력은사용자직접원칙유지.

## 사용자중단상태갱신

SSH불가로실물보정은나중에진행. 추가원격실행안함. 중단전Arducam1920×1080미리보기한장은수신완료. 오프라인보정판검출은마커0/코너0으로실제K/D계산없음. preview_detection.json참조. 사용자실측실제칸21mm·마커15.12mm는board_spec.json에기록. 다음은검출가능한화면품질부터확인하고여러자세촬영. 외부보정은실제장착후.
