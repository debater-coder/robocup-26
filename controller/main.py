import numpy as np
import rerun as rr

FIELD_BOX_SIZE = np.array([2.43, 1.82, 0.22])
PLAYING_BOX_SIZE = FIELD_BOX_SIZE - np.array([0.5, 0.5, 0])

rr.init("robocup")
rr.connect_grpc()

rr.log("field/walls", rr.Boxes3D(sizes=FIELD_BOX_SIZE, colors=[0, 255, 0]))
rr.log("field/playing_area", rr.Boxes3D(sizes=PLAYING_BOX_SIZE, colors=[255, 255, 255]))
