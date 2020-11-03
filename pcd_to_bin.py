import os
import numpy as np
import pypcd

def read_pcd(filepath):
    print("file path  {}".format(filepath))
    pc=pypcd.PointCloud.from_path(filepath)
    
    return np.array([pc.pc_data['x'],pc.pc_data['y'],pc.pc_data['z'],np.zeros(len(pc.pc_data['z']))])

def convert(pcdfolder, binfolder):
    current_path = os.getcwd()
    ori_path = os.path.join(current_path, pcdfolder)
    file_list = os.listdir(ori_path)
    des_path = os.path.join(current_path, binfolder)
    if os.path.exists(des_path):
        pass
    else:
        os.makedirs(des_path)
    for file in file_list: 
        (filename,extension) = os.path.splitext(file)
        velodyne_file = os.path.join(ori_path, filename) + '.pcd'
        pl = read_pcd(velodyne_file)
        
        pl = np.swapaxes(pl,0,1).astype(np.float32)
        velodyne_file_new = os.path.join(des_path, filename) + '.bin'
        pl.tofile(velodyne_file_new)
    
if __name__ == "__main__":  
    convert("temp_pcd/processed","bin")
