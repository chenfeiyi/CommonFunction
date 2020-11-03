import pcl
import sys
import os 
import getopt
if __name__ == "__main__":
    input_dir=''
    transfer_type=1 # 0 means ascii to binary, 1 means binary to ascii format 
    try:
        opts,args=getopt.getopt(sys.argv[1:],"ht:i:",["help","type=","indir="])
    except getopt.GetoptError:
        print("input parameter error!")
        print("Usage: -i --indir: input path;")
        print("       -h --help: help information;")
        print("       -t --type: 1: binary to ascii, 0: ascii to binary, default 1;")
    if len(opts)==0:
        print("No input path, please specify pcd file folder")
        sys.exit()
    for opt,value in opts:
        if opt in ('-h',"--help"):
            print("Usage: -i --indir: input path;")
            print("       -h --help: help information;")
            print("       -t --type: 1: binary to ascii, 0: ascii to binary, default 1;")
            print("example: test.py -i /home/pcd_folder/") 
            print("         test.py --input=/home/pcd_folder") 
            sys.exit()
        elif opt in ('-i','--indir'):
            input_dir=value

        elif opt in ('-t','--type'):
            transfer_type=int(value)
            if transfer_type!=0 or transfer_type!=1:
                print("Type error, valid type: 0: ascii to binary,1: binary to ascii")
    if len(input_dir)==0:
        print("No input path, please specify pcd file folder")
        sys.exit()

    if input_dir[-1]=='/':
        index=input_dir[:-2].rfind('/')
    else:    
        index=input_dir.rfind('/')
    output_dir=input_dir[:index]
    if transfer_type==0:
        output_dir = os.path.join(output_dir,"Binary_pcd")
    else:
        output_dir = os.path.join(output_dir,"Ascii_pcd")

    if os.path.exists(output_dir) ==False:
        os.makedirs(output_dir)

    print("Start to tranfer!")
    print("Output will save to ",output_dir)
    for filePath in os.listdir(input_dir):
        in_pcd_file_path=os.path.join(input_dir,filePath)
        out_pcd_file_path=os.path.join(output_dir,filePath)
        if in_pcd_file_path.split('.')[-1]!='pcd':
            print("Error format file: ",filePath)
            print("Skip this file!")
            continue
        pt = pcl.load_XYZI(in_pcd_file_path,'pcd')
        if transfer_type==0:
            pcl.save(pt,out_pcd_file_path,'pcd',True)
        else:
            pcl.save(pt,out_pcd_file_path,'pcd',False)

    print("Done!")