from pathlib import Path
P=Path('261028_1459/a1_work/a1_fix_src/workspace/ros2_ws/src/perception')
p=P/'include/perception/detector.hpp';s=p.read_text().replace('  int label_dark_s_max = 90;','  int label_dark_s_max = 90;\n  int label_dark_window = 31;       // 주변 평균보다 어두운 태그만 (0=끔)\n  double label_dark_c = 20.0;\n  int label_body_v_min = 15;        // 팔 라벨 검출의 그림자 몸체만, 일반 D435 HSV는 유지');p.write_text(s)
p=P/'src/color_detector.cpp';s=p.read_text();anchor='std::vector<Detection> ColorDetector::detect_labeled';head,tail=s.split(anchor,1)
tail=tail.replace('      cv::inRange(hsv, h.lower, h.upper, m);','      cv::Scalar lower = h.lower;\n      lower[2] = std::clamp(cfg_.label_body_v_min, 0, 255);\n      cv::inRange(hsv, lower, h.upper, m);',1)
tail=tail.replace('  const int tk = std::max(1, cfg_.label_tag_close);','''  if (cfg_.label_dark_window > 0) {
    cv::Mat value, mean;
    ch[2].convertTo(value, CV_32F);
    const int win = cfg_.label_dark_window | 1;
    cv::blur(value, mean, {win, win});
    dark &= value < mean - cfg_.label_dark_c;
  }
  const int tk = std::max(1, cfg_.label_tag_close);''',1);p.write_text(head+anchor+tail)
p=P/'src/perception_node.cpp';s=p.read_text();key='      ac.label_dark_s_max = static_cast<int>(declare_parameter("arm.label.dark_s_max", ac.label_dark_s_max));';assert s.count(key)==1;s=s.replace(key,key+'\n      ac.label_dark_window = static_cast<int>(declare_parameter("arm.label.dark_window", ac.label_dark_window));\n      ac.label_dark_c = declare_parameter("arm.label.dark_c", ac.label_dark_c);\n      ac.label_body_v_min = static_cast<int>(declare_parameter("arm.label.body_v_min", ac.label_body_v_min));');p.write_text(s)
p=P/'config/perception.yaml';s=p.read_text().replace('        dark_s_max: 90','        dark_s_max: 90\n        dark_window: 31        # 상대 어둠 문턱 창(px), 어두운 선반을 태그로 잡지 않음\n        dark_c: 20.0           # 주변 V평균보다 이만큼 어두워야 태그\n        body_v_min: 15         # 팔 검출만: 그림자진 옆면 포함, D435·신호 HSV 유지');p.write_text(s)
p=P/'test/test_arm_detect.cpp';s=p.read_text();s+='''
// 어두운 선반이 절대 어둠 문턱에 들어가도 라벨을 분리한다.
TEST(ArmDetect, RelativeThresholdOnDarkBackground)
{
  cv::Mat img(480,640,CV_8UC3,cv::Scalar(70,70,70));
  cylinder_top(img,{320,240},cv::Scalar(10,15,90));
  EXPECT_EQ(count_near(ColorDetector(config(true)).detect(img),RED,{320,240}),1);
}

// 밝은 윗면만 검출하면 40px 아래로 치우친다. 낮은 V의 몸체를 포함해 실제 중심 회복.
TEST(ArmDetect, ShadowSideRestoresBoxCenter)
{
  cv::Mat img(480,640,CV_8UC3,BG);
  cv::rectangle(img,{290,180},{350,300},cv::Scalar(2,4,25),cv::FILLED);
  cv::rectangle(img,{290,260},{350,300},RED_BGR,cv::FILLED);
  draw_label(img,{320,280},0);
  auto before=config(true);before.label_body_v_min=35;
  EXPECT_EQ(count_near(ColorDetector(before).detect(img),RED,{320,240}),0);
  auto after=config(true);after.label_body_v_min=15;
  EXPECT_EQ(count_near(ColorDetector(after).detect(img),RED,{320,240}),1);
}

// 팔 전용 V 문턱이 일반 D435 검출 경로에 적용되지 않는지 확인.
TEST(ArmDetect, ArmThresholdDoesNotChangeD435)
{
  cv::Mat img(480,640,CV_8UC3,BG);
  cv::rectangle(img,{290,210},{350,270},cv::Scalar(2,4,25),cv::FILLED);
  auto a=config(false),b=config(false);a.label_body_v_min=1;b.label_body_v_min=100;
  EXPECT_EQ(ColorDetector(a).detect(img).size(),ColorDetector(b).detect(img).size());
}
''';p.write_text(s)
