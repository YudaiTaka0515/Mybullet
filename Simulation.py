import pybullet as p
import pybullet_data
from pprint import pprint
# print(p.VISUAL_SHAPE_DOUBLE_SIDED)
from VisualizeClouds import visualize_cloud

# objファイル
obj_path = r"D:\Project\Mybullet\Object\buildplane.obj"
gravZ = -10


class DeformableSimulation:
    def __init__(self):
        # 物理シミュレーションへの接続
        physicsClient = p.connect(p.GUI)
        # Initialize
        p.resetSimulation(p.RESET_USE_DEFORMABLE_WORLD)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        # 重力加速度の設定
        p.setGravity(0, 0, gravZ)
        p.setPhysicsEngineParameter(sparseSdfVoxelSize=0.25)
        self.clothId = ''
        self.humanoid = ''
        self.jointIds = []
        self.paramIds = []
        self.cloth_points = 25
        self.bunny2 = ''
        useMaximalCoordinates = False
        self.a = p.loadURDF("plane.urdf", useMaximalCoordinates=useMaximalCoordinates)

        self.b = p.loadURDF("cube.urdf", [0, 2, 0], useMaximalCoordinates=useMaximalCoordinates)


    def load_soft_body(self, object_path, debug=False):
        # deformable objectの指定
        self.clothId = p.loadSoftBody(obj_path, basePosition=[0.1, -1.9, 1], scale=10, mass=1., useNeoHookean=0,
                                      useBendingSprings=1, useMassSpring=1, springElasticStiffness=1,
                                      springDampingStiffness=1,
                                      springDampingAllDirections=1, useSelfCollision=0, frictionCoeff=.5,
                                      useFaceContact=1)
        # # 固定点の設定
        for i in range(self.cloth_points):
            p.createSoftBodyAnchor(self.clothId, i, -1, -1)
            p.createSoftBodyAnchor(self.clothId, i*25, -1, -1)
            p.createSoftBodyAnchor(self.clothId, (i+1)*self.cloth_points-1, -1, -1)
        # よくわからん
        p.changeVisualShape(self.clothId, -1, flags=p.VISUAL_SHAPE_DOUBLE_SIDED)
        if debug:
            data = p.getMeshData(self.clothId, -1, flags=p.MESH_DATA_SIMULATION_MESH)
            pprint(data)

    # def load_tmp(self):
    #     self.bunny2 = p.loadURDF("plane.urdf", [0,0,-2])

    def load_robot(self):

        self.humanoid = p.loadURDF("kuka_iiwa/model.urdf", [0, -2, 0],  useFixedBase=True)  # 追加
        for j in range(p.getNumJoints(self.humanoid)):
            p.changeDynamics(self.humanoid, j, linearDamping=0, angularDamping=0)
            info = p.getJointInfo(self.humanoid, j)
            # print(info)
            jointName = info[1]
            jointType = info[2]
            if jointType == p.JOINT_PRISMATIC or jointType == p.JOINT_REVOLUTE:
                self.jointIds.append(j)
                self.paramIds.append(p.addUserDebugParameter(jointName.decode("utf-8"), -4, 4, 0))

    def simulation(self):
        p.setRealTimeSimulation(0)
        enableCollision = 1
        # p.setCollisionFilterPair(self.clothId, self.humanoid, -1, -1, enableCollision)

        clouds = []
        temp = [i for i in range(len(self.paramIds))]

        a = False
        while p.isConnected():
            p.stepSimulation()
            p.getCameraImage(320, 320)
            try:
                data = p.getMeshData(self.clothId, -1, flags=p.MESH_DATA_SIMULATION_MESH)
            except:
                break
            cloud = []
            for i in range(data[0]):
                cloud.append(data[1][i])
            clouds.append(cloud)

            for i in range(len(self.paramIds)):
                c = self.paramIds[i]

                targetPos = p.readUserDebugParameter(c)
                if temp[i] != targetPos:
                    print("--------------------")
                    # print(p.getBasePositionAndOrientation(self.humanoid))
                    # print(p.getContactPoints())

                    if i==6 and a:
                        self.create_gripper_constraints()

                    print(p.getClosestPoints(bodyA=self.humanoid, bodyB=self.clothId, distance=0.0001))
                    print(p.getClosestPoints(bodyA=self.humanoid, bodyB=self.a, distance=1))
                    print(p.getContactPoints(bodyA=self.humanoid, bodyB=self.a))
                temp[i] = targetPos
                p.setJointMotorControl2(self.humanoid, self.jointIds[i], p.POSITION_CONTROL, targetPos, force=5 * 240.)
            # wa, _ = p.getBasePositionAndOrientation(self.clothId)
            # a = p.applyExternalForce(objectUniqueId=self.clothId, linkIndex=-1, forceObj=(0, 20, 30),
            #                          posObj=wa, flags=p.WORLD_FRAME)
            # print(a)
            # print(p.performCollisionDetection())
            p.setGravity(0, 0, gravZ)
            a = True


        return clouds

    def create_gripper_constraints(self):
        p.createConstraint(
            parentBodyUniqueId = self.humanoid,
            parentLinkIndex = 6,
            childBodyUniqueId = self.clothId,
            childLinkIndex = -1,
        )



if __name__ == '__main__':
    sim = DeformableSimulation()
    sim.load_soft_body(obj_path)
    sim.load_robot()
    clouds = sim.simulation()
    # visualize_cloud(clouds)








