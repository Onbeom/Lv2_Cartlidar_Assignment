#!/bin/bash
# A1 판정용 합친 빌드: fix/perception-arm-lock-gate 작업 트리(tag-align 위) + fix/perception-arm-detect 변경(dev 위)
set -e
S=/tmp/claude-1000/-home-pa4-git-lv2-next-module/53f1d6b5-f824-4b3c-9008-e4ebfdcc61fe/scratchpad
LG=~/git/Lv2_Cartlidar_Assignment_lockgate; AD=~/git/Lv2_Cartlidar_Assignment_armdetect
SRC=$S/a1_fix_src
rm -rf "${SRC:?}"; mkdir -p "$SRC/workspace/ros2_ws"
cp -r $LG/workspace/ros2_ws/src "$SRC/workspace/ros2_ws/"
P=$SRC/workspace/ros2_ws/src/perception; A=$AD/workspace/ros2_ws/src/perception
cp $A/include/perception/detector.hpp $P/include/perception/detector.hpp
cp $A/src/color_detector.cpp $P/src/color_detector.cpp
cp $A/test/test_arm_detect.cpp $P/test/test_arm_detect.cpp
python3 - "$P" "$A" <<'PY'
import sys
P,A=sys.argv[1],sys.argv[2]
def ed(f,a,b):
    s=open(f).read(); assert s.count(a)==1,(f,a); open(f,'w').write(s.replace(a,b))
ed(P+'/CMakeLists.txt','test/test_arm_lock.cpp)','test/test_arm_lock.cpp test/test_arm_detect.cpp)')
ya=open(A+'/config/perception.yaml').read()
ed(P+'/config/perception.yaml','    detector:\n      type: auto',ya[ya.index('      label:'):ya.index('    detector:\n')]+'    detector:\n      type: auto')
ed(P+'/src/perception_node.cpp','      const auto dets = detector_->detect(small);','      const auto dets = (arm_detector_ ? *arm_detector_ : *detector_).detect(small);')
an=open(A+'/src/perception_node.cpp').read()
blk=an[an.index('    // 팔 카메라(ARM_ALIGN): 색 검출기면'):an.index('      arm_detector_ = std::make_unique<ColorDetector>(ac);\n    }\n')+len('      arm_detector_ = std::make_unique<ColorDetector>(ac);\n    }\n')]
key='    split_on_ = split_on_ && detector_->name() != "tensorrt";   // 깊이 나누기는 색 검출기 덩어리에만\n'
ed(P+'/src/perception_node.cpp',key,key+blk)
ed(P+'/src/perception_node.cpp','  std::unique_ptr<Detector> detector_;\n','  std::unique_ptr<Detector> detector_;\n  std::unique_ptr<Detector> arm_detector_;   // ARM_ALIGN 전용 (없으면 detector_)\n')
PY
source /opt/ros/lyrical/setup.bash
mkdir -p $S/a1_fix_build && cd $S/a1_fix_build
colcon build --base-paths $SRC/workspace/ros2_ws/src --packages-up-to perception --cmake-args -DCMAKE_BUILD_TYPE=Release > build.log 2>&1
./build/perception/test_perception_logic 2>&1 | tail -2
