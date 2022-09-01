from distutils.command.build import build
import os
import time
import numpy as np
import pybullet as p
import pybullet_data
import pybullet_industrial as pi

x_ring = [[0.030395136778115502, 0.0972644376899696, 0.1762917933130699, 0.11246200607902736], [0.15501519756838905, 0.24012158054711247, 0.2978723404255319, 0.2066869300911854], [0.3130699088145897, 0.41641337386018235, 0.4376899696048632, 0.3343465045592705], [0.49544072948328266, 0.5987841945288753, 0.5805471124620061, 0.47720364741641336], [0.668693009118541, 0.7629179331306991, 0.7112462006079028, 0.6200607902735562], [0.8237082066869301, 0.8936170212765957, 0.8145896656534954, 0.7477203647416414], [0.9300911854103343, 0.9696048632218845, 0.8723404255319149, 0.8358662613981763], [0.9848024316109423, 0.9848024316109423, 0.8814589665653495, 0.878419452887538], [0.9726443768996961, 0.939209726443769, 0.8389057750759878, 0.8753799392097265]]
y_ring = [[0.26122448979591834, 0.1877551020408163, 0.2476190476190476, 0.32108843537414966], [0.14421768707482993, 0.09523809523809523, 0.1768707482993197, 0.22312925170068026], [0.07074829931972788, 0.05170068027210884, 0.14421768707482993, 0.16326530612244897], [0.05170068027210884, 0.06802721088435373, 0.16054421768707483, 0.14421768707482993], [0.08979591836734693, 0.13605442176870747, 0.21768707482993196, 0.17142857142857143], [0.1768707482993197, 0.25034013605442174, 0.31020408163265306, 0.23945578231292516], [0.31020408163265306, 0.3972789115646258, 0.4272108843537415, 0.34285714285714286], [0.46802721088435373, 0.5605442176870747, 0.5605442176870747, 0.46802721088435373], [0.6258503401360543, 0.7156462585034014, 0.6829931972789115, 0.5931972789115646]]

w_x = [0.6930091185410334, 0.6930091185410334, 0.3768996960486322, 0.6899696048632219, 0.6930091185410334, 0.3829787234042553, 0.6930091185410334, 0.6930091185410334, 0.25835866261398177, 0.2553191489361702, 0.5714285714285714, 0.2553191489361702, 0.2553191489361702]
w_y = [0.35918367346938773, 0.5006802721088435, 0.563265306122449, 0.6312925170068027, 0.7918367346938775, 0.8598639455782312, 0.9251700680272108, 1.0530612244897959, 0.9360544217687075, 0.780952380952381, 0.710204081632653, 0.636734693877551, 0.47891156462585033]

b_outer_x= [0.6519756838905775, 0.6671732522796353, 0.682370820668693, 0.6945288753799392, 0.7006079027355623, 0.7036474164133738, 0.7006079027355623, 0.6884498480243161, 0.6732522796352584, 0.6580547112462006, 0.6337386018237082, 0.6094224924012158, 0.5820668693009119, 0.5547112462006079, 0.5212765957446809, 0.4908814589665653, 0.45440729483282677, 0.4209726443768997, 0.39361702127659576, 0.3601823708206687, 0.32674772036474165, 0.30243161094224924, 0.27710234575358567, 0.26190477736452794, 0.2497467226532817, 0.24366769529765864, 0.24670720897547016, 0.25278623633109326, 0.26494429104233946, 0.28014185943139724, 0.29533942782045497, 0.31053699620951275, 0.33485310563200515, 0.31661602356513585, 0.29533942782045497, 0.27102331839796256, 0.25582575000890484, 0.25582575000890484, 0.25582575000890484, 0.8839918791704264, 0.8839918791704264]
b_outer_y = [1.2371882224569515, 1.2507936646338222, 1.2725623721168153, 1.2997732564705569, 1.3297052292596725, 1.3623582904841622, 1.3895691748379038, 1.4249433244977678, 1.446712031980761, 1.46575965102838, 1.484807270075999, 1.4956916238174955, 1.5120181544297406, 1.520181419735863, 1.5256235966066114, 1.5283446850419855, 1.5283446850419855, 1.522902508171237, 1.5174603313004889, 1.5065759775589922, 1.4875283585113732, 1.4739229163345025, 1.4476190476190476, 1.417687074829932, 1.385034013605442, 1.3578231292517007, 1.327891156462585, 1.3006802721088435, 1.276190476190476, 1.254421768707483, 1.238095238095238, 1.2244897959183672, 1.2108843537414966, 1.2108843537414966, 1.2108843537414966, 1.2108843537414966, 1.2136054421768707, 1.1945578231292515, 1.0884353741496597, 1.0870748299319728, 1.2231292517006802]

b_inner_y = [1.22154196395355, 1.2242630523889242, 1.2269841408242983, 1.2324263176950467, 1.240589583001169, 1.2487528483072916, 1.2623582904841624, 1.2814059095317814, 1.2977324401440262, 1.3222222360623936, 1.3439909435453867, 1.3603174741576316, 1.3739229163345024, 1.3793650932052508, 1.382086181640625, 1.3875283585113731, 1.3875283585113731, 1.384807270075999, 1.382086181640625, 1.3712018278991283, 1.363038562593006, 1.3521542088515093, 1.3358276782392644, 1.3167800591916454, 1.2977324401440262, 1.2759637326610331, 1.256916113613414, 1.2433106714365434, 1.2324263176950467, 1.2242630523889242]
b_inner_x= [0.46656534954407297, 0.4969604863221884, 0.5121580547112462, 0.5334346504559271, 0.5455927051671733, 0.560790273556231, 0.5729483282674772, 0.5820668693009119, 0.5881458966565349, 0.5851063829787234, 0.5759878419452887, 0.5638297872340425, 0.5455927051671733, 0.5303951367781155, 0.5182370820668692, 0.5, 0.4726443768996961, 0.4513677811550152, 0.4331306990881459, 0.40577507598784196, 0.3905775075987842, 0.378419452887538, 0.3662613981762918, 0.3601823708206687, 0.3601823708206687, 0.3662613981762918, 0.38145896656534956, 0.39969604863221886, 0.4209726443768997, 0.44224924012158057]

k_y = [1.5752834605521895, 1.711337882320897, 1.7086167938855228, 1.8229025081712371, 1.9725623721168153, 1.8365079503481079, 1.9807256374229378, 1.8256235966066112, 1.711337882320897, 1.711337882320897, 1.5752834605521895]
k_x = [0.8860182370820668, 0.8860182370820668, 0.5121580547112462, 0.6945288753799392, 0.6914893617021276, 0.4969604863221884, 0.2629179331306991, 0.25987841945288753, 0.4696048632218845, 0.25987841945288753, 0.25987841945288753]

def build_path(x_data, y_data,number_of_steps):
    z_data=[]
    
    i=0
    path_list=[]
    while i<len(x_data):
        if i<len(x_data)-1:
            sub_path=pi.linear_interpolation(np.array([2.5-x_data[i], y_data[i]-1, height]),np.array([2.5-x_data[i+1], y_data[i+1]-1, height]),number_of_steps)
        if i==len(x_data)-1:
            sub_path=pi.linear_interpolation(np.array([2.5-x_data[i], y_data[i]-1, height]),np.array([2.5-x_data[0], y_data[0]-1, height]),number_of_steps)
        path_list.append(sub_path)
        i+=1
    path=path_list[0]
    i=1
    while i<len(path_list):
        path.append(path_list[i])
        i+=1
        
    path.draw()
    return path

def scale_paths(path):
    
    path_copy_list=[]
    max_x=np.max(path.positions[0])
    min_x=np.min(path.positions[0])
    width_x=np.abs(max_x-min_x)
    
    max_y=np.max(path.positions[1])
    min_y=np.min(path.positions[1])
    width_y=np.abs(max_y-min_y)

    #assuming that path stays at the same height
    height=np.mean(path.positions[2])

    scale_list=[0.2,0.4,0.6]
    center_point=np.array([min_x+width_x/2, min_y+width_y/2, height])
    for scale in scale_list:
        
        path_copy=path
        path_copy.translate(-center_point)
        
        path_copy.positions*=scale
        path_copy.translate(center_point)
        
        path_copy.draw()
        path_copy_list.append(path_copy) 
    return path_copy_list

def build_smaller_paths(path, scale):
    x_list=[]
    y_list=[]
    z_list=[]
    
    for x,y,z in zip(path.positions[0],path.positions[1],path.positions[2]):
        x_list.append(x)
        y_list.append(y)
        z_list.append(z)
    i=0
    new_x_list=[]
    new_y_list=[]
    new_z_list=[]
    while i<len(x_list)-1:
        new_z_list.append(height)
    i=0
    while i<len(x_list)-1:
        point=np.array([x_list[i], y_list[i], z_list[i]])
        diff_vector=point-np.array([x_list[i+1], y_list[i+1], z_list[i+1]])
        theta=np.pi /4
        x=np.cos(theta)*diff_vector[0]-np.sin(theta)*diff_vector[1]
        y=np.sin(theta)*diff_vector[0]+np.cos(theta)*diff_vector[1]
        new_x_list.append(x)
        new_y_list.append(y)
        i+=1
    new_points=np.array([new_x_list, new_y_list, new_z_list])
    new_path=pi.ToolPath(new_points)
    new_path.draw()

    return new_path
    



if __name__ == "__main__":
    dirname = os.path.dirname(__file__)
    urdf_file1 = os.path.join(dirname,
                              'robot_descriptions', 'comau_NJ290_3-0_m.urdf')
    urdf_file2 = os.path.join(dirname,
                              'robot_descriptions', 'milling_head.urdf')

    physics_client = p.connect(p.GUI)
    p.setPhysicsEngineParameter(numSolverIterations=5000)

    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    monastryId = p.createCollisionShape(p.GEOM_MESH,
                                        fileName="samurai_monastry.obj",
                                        flags=p.GEOM_FORCE_CONCAVE_TRIMESH)

    orn = p.getQuaternionFromEuler([1.5707963, 0, 0])
    p.createMultiBody(0, monastryId, baseOrientation=orn)
    
    height=0.53
    start_orientation = p.getQuaternionFromEuler([0, 0, 0])
    robot = pi.RobotBase(urdf_file1, [0, 0, 0], start_orientation)

    remover_properties = {'maximum distance': height-0.505,
                          'opening angle': 0,
                          'number of rays': 1}
    remover = pi.Remover(
        urdf_file2, [2, 0, 2], start_orientation, remover_properties)
    
    remover.couple(robot, 'link6')
    
    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
    pi.spawn_material_block([1.5,-1,0], [1,2,0.5], pi.MetalVoxel, {'particle size': 0.5, 'color': [0, 1, 0.415686, 1]})
    
    pi.spawn_material_block([1.5,-1,0.5], [1,2, height-0.5], pi.MetalVoxel, {'particle size': (height-0.5), 'color': [1,1,1,1]})
    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
    
    
    path_list=[]
    for sub_x, sub_y in zip(x_ring, y_ring):
        sub_path=build_path(sub_x, sub_y,6)    
        path_list.append(sub_path)

    w_path=build_path(w_x, w_y,15)
    b_outer_path=build_path(b_outer_x, b_outer_y,8)
    b_inner_path=build_path(b_inner_x, b_inner_y,2)
    k_path=build_path(k_x, k_y, 15)

    path_list+=(w_path, b_outer_path, b_inner_path, k_path)
    
    
    for sub_path in path_list:
        for _ in range(20):
            remover.set_tool_pose(*sub_path.get_start_pose())
            for _ in range(50):
                p.stepSimulation()
    
        for position, orientation, tool_path in sub_path:
            remover.set_tool_pose(position, orientation)
            for _ in range(20):
                p.stepSimulation()
            remover.remove()