import sys,unittest
sys.path.insert(0,'Cartlidar/sim_render')
from dice_multiview import DiceObservations
CELLS=[[.06*(i%3-1),.06*(1-i//3),.07] for i in range(9)]
EXPECT=[0]*9;EXPECT[4]=1

def puck(tag=0,pos=(0,0,.10),color='red',shape='cylinder'):
    return dict(tag_id=tag,position=pos,color=color,shape=shape)
class Safety(unittest.TestCase):
    def test_same_id_across_views_is_one(self):
        a=DiceObservations(CELLS,EXPECT,'cylinder')
        a.observe([puck()],1,'a',range(9));a.observe([puck(pos=(.001,0,.10))],2,'b',range(9))
        r=a.finish();self.assertTrue(r['passed']);self.assertEqual(len(r['pucks']),1);self.assertEqual(r['pucks'][0]['n_obs'],2)
    def test_distinct_ids_same_cell_are_multiple(self):
        a=DiceObservations(CELLS,EXPECT,'cylinder');a.observe([puck(),puck(tag=1)],1,'a',range(9))
        self.assertEqual(a.finish()['error'][4],3);self.assertFalse(a.finish()['passed'])
    def test_same_id_far_apart_rejected(self):
        a=DiceObservations(CELLS,EXPECT,'cylinder');a.observe([puck()],1,'a',range(9));a.observe([puck(pos=(.06,0,.1))],2,'b',range(9));self.assertFalse(a.finish()['observed'])
    def test_unseen_cell_not_empty(self):
        a=DiceObservations(CELLS,[0]*9,'cylinder');a.observe([],1,'a',range(8));self.assertFalse(a.finish()['observed']);self.assertFalse(a.finish()['passed'])
    def test_reused_observation_rejected(self):
        a=DiceObservations(CELLS,EXPECT,'cylinder');a.observe([puck()],1,'a',range(9))
        with self.assertRaises(ValueError):a.observe([puck()],1,'a',range(9))
    def test_conflicting_attributes_rejected(self):
        a=DiceObservations(CELLS,EXPECT,'cylinder');a.observe([puck()],1,'a',range(9));a.observe([puck(color='green')],2,'b',range(9));self.assertFalse(a.finish()['observed'])
    def test_red_group_does_not_require_shape(self):
        a=DiceObservations(CELLS,EXPECT,'cylinder');a.observe([puck(shape=None)],1,'a',range(9));self.assertTrue(a.finish()['passed'])
    def test_yellow_group_does_not_require_color(self):
        want=[0]*9;want[4]=3
        a=DiceObservations(CELLS,want,'cylinder');a.observe([puck(color=None)],1,'a',range(9));self.assertTrue(a.finish()['passed'])
if __name__=='__main__':unittest.main()
