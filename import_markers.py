import Metashape as  ms
import lecture as lec
import torch
import argparse


def create_metashape_project(project_path, gopro_path):
    doc = ms.Document()
    doc.open(project_path)
    local_chunk = doc.chunk
    
    obs = lec.lecture_marker(gopro_path + "\mrkrtot.txt")
    
    num_cam = int(len(local_chunk.cameras)/2)
    marker = local_chunk.addMarker()
    for i in range(num_cam):
        camera = local_chunk.cameras[i+num_cam]
        print(camera)
        try :
            obs[i]
            for j in range( len(obs[i]) ):
               print(obs[i][j])
               marker = local_chunk.addMarker()
               marker.projections[local_chunk.cameras[i+num_cam]] = ms.Marker.Projection((obs[i][j]['x'],obs[i][j]['y']), True)
               marker.reference.location = ms.Vector([obs[i][j]['X'],obs[i][j]['Y'],obs[i][j]['Z']])
        except: 
            print(i)    
    
    ## import photos and cameras ... ##
    doc.save(project_path)

if __name__ == "__main__":
    
    parser = argparse.ArgumentParser(description="Paths")
    parser.add_argument("--project_path", type=str,  default="data/*.png", required=True, help="Images path")
    parser.add_argument("--gopro_path", type=str, default=r"checkpoints_G_coord_resnet50\best_ckpt.pt", required=True, help="checkpoint path")

    args = parser.parse_args()

    create_metashape_project(
        project_path = args.project_path,
        gopro_path = args.gopro_path,
    ) 
    
    
    
    
    