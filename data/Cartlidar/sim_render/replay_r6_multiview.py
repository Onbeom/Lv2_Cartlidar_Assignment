"""실제 ROS 태그검출 → 별도 여러시야노드 → 그룹별 R6 판정. 정답 퍽은 채점에만 사용."""
import argparse,json,time
from pathlib import Path
import rclpy
from std_msgs.msg import String
from robot_interfaces.srv import SetPerceptionMode
from replay_r6 import DiceReplayer

class Replayer(DiceReplayer):
    def __init__(self,a):
        super().__init__(a);self.results=[]
        self.command_pub=self.create_publisher(String,'/perception/dice_session',10)
        self.create_subscription(String,'/perception/dice_multiview',lambda m:self.results.append(json.loads(m.data)),30)
    def command(self,data,want):
        before=len(self.results);m=String();m.data=json.dumps(data);self.command_pub.publish(m)
        start=time.monotonic()
        while time.monotonic()-start<5:
            self.spin_for(.05)
            for row in self.results[before:]:
                if row.get('state')==want:return row
        raise RuntimeError(f'여러시야노드응답없음 {want}')

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--render',type=Path,required=True);ap.add_argument('--out',type=Path,required=True)
    ap.add_argument('--t0',type=float,default=1000.0);ap.add_argument('--start-delay',type=float,default=1.0);ap.add_argument('--qos-depth',type=int,default=10);ap.add_argument('--settle',type=float,default=.10);ap.add_argument('--max-wait',type=float,default=2.0)
    a=ap.parse_args();a.out.mkdir(parents=True,exist_ok=True);rclpy.init();n=Replayer(a);n.set_clock(a.t0);n.spin_for(1.0);n.set_mode(SetPerceptionMode.Request.READ_TAG)
    groups={}
    for p in n.meta['poses']:groups.setdefault((p['group_id'],p['light_level']),[]).append(p)
    rows=[];t=a.t0+1
    try:
        for (group,level),poses in groups.items():
            reference=poses[0]
            n.command({'action':'begin','start_stamp':t,'cells':reference['cells'],'expect':reference['expect'],'yellow_shape':{1:'cylinder',2:'cuboid'}[reference['yellow_shape']],'tolerance':.010},'ready')
            missing=[]
            for p in poses:
                start=len(n.results);n.set_clock(t);n.publish(p,t);wall=time.monotonic();got=None;tick=0
                while time.monotonic()-wall<a.max_wait:
                    tick+=1;n.set_clock(t+min(tick/30.,.2666666667))
                    n.spin_for(.05)
                    got=next((r for r in n.results[start:] if r.get('state')=='observed' and abs(r['stamp']-t)<1e-5),None)
                    if got:break
                if got is None:missing.append(p['name'])
                t+=1.;n.set_clock(t)
            result=n.command({'action':'finish'},'result')
            match=not missing and result['observed'] and result['passed']==reference['expected_passed'] and result['wrong']==reference['expected_wrong'] and result['error']==reference['expected_error']
            row={'group':group,'light':level,'kind':reference['kind'],'match':match,'missing_frames':missing,'expected_error':reference['expected_error'],'expected_passed':reference['expected_passed'],'result':result,'placed_tags':sorted(p['tag_id'] for p in reference['pucks'])};rows.append(row)
            with (a.out/'rows.jsonl').open('a') as f:f.write(json.dumps(row,ensure_ascii=False)+'\n')
            print(group,level,match,'오류',result['error'],'관측ID',[p['tag_id'] for p in result['pucks']],'미관측칸',sorted(set(range(9))-set(result['covered_cells'])),flush=True)
            t+=1;n.set_clock(t)
    finally:n.set_mode(SetPerceptionMode.Request.OFF);n.destroy_node();rclpy.shutdown()
    summary={'groups':len(rows),'match':sum(r['match'] for r in rows),'pass':bool(rows) and all(r['match'] for r in rows),'mismatches':[r['group'] for r in rows if not r['match']]};(a.out/'summary.json').write_text(json.dumps(summary,indent=2));print(summary)
if __name__=='__main__':main()
