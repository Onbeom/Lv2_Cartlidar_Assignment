"""팀 저장소에서 외부 월드 파일 없이 실행 가능한 등록부 안전 회귀 시험."""
import math
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from perception_survey.core import Survey,BOX_CENTERS

def filled():
    s=Survey()
    for frame in range(2):
        for box,(x,y) in enumerate(BOX_CENTERS):
            angle=math.radians((box-3)*8)
            for side in (-1,1):
                tid=2*box+(side+1)//2
                assert s.observe({'x':x+side*.04*math.cos(angle),'y':y+side*.04*math.sin(angle),
                    'id':tid,'color':'red' if tid%2 else 'green','shape':'cylinder','pose':'standing',
                    'camera':'arm','confidence':.95},10.+frame)
    return s

def test_complete_fixed_twelve_slots():
    r=filled().snapshot()
    assert r['complete'] and len(r['slots'])==12
    assert sorted(t['id'] for t in r['slots'])==list(range(12))

def test_atomic_finish_freezes(tmp_path):
    s=filled();p=tmp_path/'puck_survey.json';r=s.finish(p)
    import json
    assert json.loads(p.read_text())==r and s.frozen

def test_incomplete_does_not_replace_valid_file(tmp_path):
    p=tmp_path/'puck_survey.json';p.write_text('previous')
    import pytest
    with pytest.raises(ValueError):Survey().finish(p)
    assert p.read_text()=='previous'

def test_conflict_is_not_hidden_by_majority():
    s=filled();t=s.snapshot()['slots'][0]
    for k in range(3):
        assert s.observe({'x':t['x_mm']/1000,'y':t['y_mm']/1000,'id':t['id'],'color':'red','shape':'cylinder','pose':'standing','camera':'arm','confidence':1.},20.+k)
    assert not s.snapshot()['complete']

def test_duplicates_not_independent_observations():
    s=Survey();x,y=BOX_CENTERS[0];o={'x':x-.04,'y':y,'id':0,'color':'green','shape':'cylinder','pose':'standing','camera':'arm','confidence':1.}
    assert s.observe(o,10.) and not s.observe(o,10.)
    assert s.snapshot()['slots'][0]['n_obs']==1

def test_reset_has_new_survey_id():
    s=filled();old=s.snapshot()['survey_id'];s.reset()
    assert s.snapshot()['survey_id']!=old and not s.snapshot()['complete']

def test_merge_gate_cannot_bridge_neighbor_slots():
    import pytest
    with pytest.raises(ValueError):Survey(gate_m=.04)

def test_invalid_yaw_cannot_partially_modify_state():
    s=Survey();x,y=BOX_CENTERS[0]
    assert not s.observe({'x':x-.04,'y':y,'id':0,'camera':'arm','confidence':1.,'yaw_deg':'bad'},10.)
    assert all(t['n_obs']==0 for t in s.snapshot()['slots'])


def test_tag_ring_coverage_uses_explicit_parameter():
    s=Survey(tagged_min_confidence=.4);x,y=BOX_CENTERS[0]
    o={'x':x-.04,'y':y,'id':0,'color':'red','shape':'cylinder','pose':'standing','camera':'arm','confidence':.5}
    assert s.observe(o,10.) and s.observe(o,11.)
    slot=s.snapshot()['slots'][0]
    assert (slot['color'],slot['shape'],slot['id'])==('red','cylinder',0)
    assert not s.snapshot()['complete']

def test_default_threshold_and_non_tag_threshold_are_preserved():
    x,y=BOX_CENTERS[0]
    for s,tid,camera in [(Survey(),0,'arm'),(Survey(tagged_min_confidence=.4),-1,'front_d435')]:
        assert s.observe({'x':x-.04,'y':y,'id':tid,'color':'red','shape':'cylinder','pose':'standing','camera':camera,'confidence':.5},10.)
        assert s.snapshot()['slots'][0]['color'] is None

def test_low_tag_score_and_conflicting_color_still_refuse():
    x,y=BOX_CENTERS[0];s=Survey(tagged_min_confidence=.4)
    o={'x':x-.04,'y':y,'id':0,'color':'red','shape':'cylinder','pose':'standing','camera':'arm','confidence':.2}
    assert s.observe(o,10.) and s.snapshot()['slots'][0]['color'] is None
    for stamp,color in [(11.,'red'),(12.,'red'),(13.,'green')]:
        assert s.observe(dict(o,color=color,confidence=.8),stamp)
    assert s.snapshot()['slots'][0]['color'] is None and not s.snapshot()['complete']

def test_invalid_tag_score_threshold_refused():
    import pytest
    for value in [-.1,1.1,float('nan'),float('inf')]:
        with pytest.raises(ValueError):Survey(tagged_min_confidence=value)

def test_arm_tag_shape_wins_over_one_wrong_d435_vote():
    # 앞 D435 YOLO 가 한 장에서 원기둥을 직육면체로 본 경우(F1 · 후보 C, seed 103): 팔 태그 관측이 있으면 그 표로 정한다
    s=filled();t=s.snapshot()['slots'][0]
    assert s.observe({'x':t['x_mm']/1000,'y':t['y_mm']/1000,'id':-1,'color':'green','shape':'cuboid','camera':'front_d435','confidence':.79},20.)
    r=s.snapshot();slot=r['slots'][0]
    assert slot['shape']=='cylinder' and r['complete']

def test_d435_votes_used_when_no_arm_tag_vote():
    x,y=BOX_CENTERS[0];s=Survey(tagged_min_confidence=.4)
    for stamp in (10.,11.):
        assert s.observe({'x':x-.04,'y':y,'id':0,'color':'red','shape':'cylinder','pose':'standing','camera':'arm','confidence':.2},stamp)
    for stamp,shape in [(12.,'cuboid'),(13.,'cuboid')]:
        assert s.observe({'x':x-.04,'y':y,'id':-1,'color':'red','shape':shape,'camera':'front_d435','confidence':.9},stamp)
    assert s.snapshot()['slots'][0]['shape']=='cuboid'
    assert s.observe({'x':x-.04,'y':y,'id':-1,'color':'red','shape':'cylinder','camera':'front_d435','confidence':.9},14.)
    assert s.snapshot()['slots'][0]['shape'] is None

def test_revisit_clears_only_problem_slot_then_completes():
    # 한 칸이 엇갈리면 조사 전체를 다시 시작하지 않고 그 칸만 비운다. 다른 칸의 관측은 그대로 남는다
    s=filled();t=s.snapshot()['slots'][0]
    assert s.observe({'x':t['x_mm']/1000,'y':t['y_mm']/1000,'id':t['id'],'color':'red','shape':'cylinder','pose':'standing','camera':'arm','confidence':1.},20.)
    r=s.snapshot()
    assert r['revisit']==['1:-1'] and [x['status'] for x in r['slots']].count('confirmed')==11
    before=[x['n_obs'] for x in r['slots'][1:]]
    todo,unconfirmed=s.revisit()
    assert [x['slot'] for x in todo]==['1:-1'] and todo[0]['revisits']==1 and unconfirmed==[]
    assert 'color_unknown_or_conflict' in todo[0]['problems']
    r=s.snapshot()
    assert r['slots'][0]['n_obs']==0 and [x['n_obs'] for x in r['slots'][1:]]==before
    for stamp in (21.,22.):
        assert s.observe({'x':t['x_mm']/1000,'y':t['y_mm']/1000,'id':t['id'],'color':t['color'],'shape':'cylinder','pose':'standing','camera':'arm','confidence':1.},stamp)
    r=s.snapshot()
    assert r['complete'] and r['slots'][0]['status']=='confirmed' and r['slots'][0]['revisits']==1

def test_revisit_limit_marks_unconfirmed():
    # 한도만큼 다시 봐도 못 채운 칸은 미확인(unconfirmed)으로 남고 더 비우지 않는다. 낮은 점수 D435 관측은 여전히 버린다
    s=Survey(max_revisits=1);x,y=BOX_CENTERS[0]
    low={'x':x-.04,'y':y,'id':-1,'color':'red','shape':'cylinder','camera':'front_d435','confidence':.55}
    assert s.observe(low,10.) and s.snapshot()['slots'][0]['color'] is None
    todo,_=s.revisit()
    assert len(todo)==12
    assert s.observe(dict(low),11.)
    r=s.snapshot()
    assert r['slots'][0]['status']=='unconfirmed' and len(r['unconfirmed'])==12 and r['revisit']==[] and not r['complete']
    assert s.revisit()==([],r['unconfirmed']) and s.snapshot()['slots'][0]['n_obs']==1

def test_duplicate_id_sends_both_slots_to_revisit():
    s=Survey();x,y=BOX_CENTERS[0]
    for stamp in (10.,11.):
        for side in (-1,1):
            assert s.observe({'x':x+side*.04,'y':y,'id':0,'camera':'arm','confidence':1.},stamp)
    r=s.snapshot()
    assert {'1:-1','1:1'}<=set(r['revisit'])
    assert all('duplicate_id:0' in r['slots'][i]['problems'] for i in (0,1))

def test_revisit_refused_after_finish(tmp_path):
    import pytest
    s=filled();s.finish(tmp_path/'puck_survey.json')
    with pytest.raises(ValueError):s.revisit()
    for value in [-1,1.5,True]:
        with pytest.raises(ValueError):Survey(max_revisits=value)

def test_free_layout_random_positions():
    """free_layout (사용자 규칙 11-16): 퍽이 박스 중심에서 3 ~ 6 cm 아무 자리 -> 박스로 묶어 12 칸 완료, 잰 자리 그대로, 박스 요 안 잼"""
    s=Survey(free_layout=True);rng=__import__('random').Random(1);truth={}
    for box,(x,y) in enumerate(BOX_CENTERS):
        a,b=rng.uniform(-math.pi,math.pi),rng.uniform(.03,.06)
        p=(x+b*math.cos(a),y+b*math.sin(a));q=(x-.05*math.cos(a+.4),y-.05*math.sin(a+.4))   # 두 퍽 중심 거리 >= 35 mm
        truth[2*box]=p;truth[2*box+1]=q
    for frame in range(2):
        for tid,(x,y) in truth.items():
            n=.003*(1 if frame else -1)
            assert s.observe({'x':x+n,'y':y-n,'id':tid,'color':'red','shape':'cylinder','pose':'standing','camera':'arm','confidence':.95},10.+frame)
    r=s.snapshot()
    assert r['complete'],r['errors']
    for t in r['slots']:
        x,y=truth[t['id']];assert math.hypot(t['x_mm']/1000-x,t['y_mm']/1000-y)<1e-6
    assert not any(b['yaw_measured'] for b in r['boxes'])

def test_free_layout_rejects():
    s=Survey(free_layout=True);x,y=BOX_CENTERS[0]
    o=lambda dx,dy,i:{'x':x+dx,'y':y+dy,'id':i,'color':'red','shape':'cylinder','pose':'standing','camera':'arm','confidence':.95}
    assert not s.observe(o(.2,0,0),1.) and s.rejected=={'outside_box':1}
    assert s.observe(o(.05,0,0),1.) and s.observe(o(-.03,.02,1),1.)
    assert not s.observe(o(0,-.05,2),2.) and s.rejected['outside_slot']==1   # 박스에 퍽 2 개가 이미 있음
    assert s.observe(o(.06,.005,0),2.)                                          # 같은 퍽 (20 mm 안)
