import cnoid.Base as base
import cnoid.BodyPlugin as bodyp
import cnoid.Body as body

#Get root singleton, simulator, robots
root = base.RootItem.instance()
simulator = root.findItem("World/AISTSimulator")
robot = simulator.findItem("SampleRobot")
floor = simulator.findItem("Floor")

link_hrp2 = robot.body().link(3)
link_floor = floor.body().link(0)

simulator.addLinkConnection(robot, link_hrp2, [0, 0, 1.],
                            floor, link_floor, [0, 0, 0])
