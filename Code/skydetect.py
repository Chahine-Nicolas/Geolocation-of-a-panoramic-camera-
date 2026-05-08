import os
import glob
import time
import torch
import numpy as np
import cv2
from cv2.ximgproc import guidedFilter
from networks import define_G
import argparse

# -----------------------------
# Model loading
# -----------------------------
def load_model(checkpoint_path, device="cpu"):
    device = torch.device(device)

    model = define_G(input_nc=3, output_nc=1, ngf=64, netG="coord_resnet50")
    #checkpoint = torch.load(checkpoint_path, map_location=device)
    checkpoint = torch.load(checkpoint_path, map_location=device, weights_only=False)
    model.load_state_dict(checkpoint['model_G_state_dict'])
    model.to(device)
    model.eval()

    return model, device


# -----------------------------
# Image preprocessing
# -----------------------------
def preprocess_image(img_path):
    img = cv2.imread(img_path)
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    img = img.astype(np.float32) / 255.0
    h, w, _ = img.shape
    in_size_w = 4608 // 6  # 384
    in_size_h = 3456 // 6  # 576

    img_resized = cv2.resize(img, (in_size_w, in_size_h))
    tensor = torch.tensor(img_resized).permute(2, 0, 1).unsqueeze(0)

    return img, tensor, (h, w)


# -----------------------------
# Model inference
# -----------------------------
def predict_mask(model, tensor, original_size, device):
    h, w = original_size

    with torch.no_grad():
        pred = model(tensor.to(device))
        pred = torch.nn.functional.interpolate(
            pred, (h, w), mode='bicubic', align_corners=False
        )

        pred = pred[0].permute(1, 2, 0)
        pred = torch.cat([pred] * 3, dim=-1)
        pred = pred.cpu().numpy()
        pred = np.clip(pred, 0.0, 1.0)

    return pred


# -----------------------------
# Post-processing
# -----------------------------
def refine_mask(img, pred, r=20, eps=0.01, threshold=0.5):
    refined = guidedFilter(img[:, :, 2], pred[:, :, 0], r, eps)
    mask = np.clip(refined, 0, 1)

    binary = ((mask > threshold) * 255).astype(np.uint8)

    contours, _ = cv2.findContours(binary, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

    if not contours:
        return binary

    largest = max(contours, key=cv2.contourArea)

    final_mask = np.zeros(binary.shape, np.uint8)
    cv2.drawContours(final_mask, [largest], -1, 255, -1)

    return final_mask


# -----------------------------
# Main pipeline
# -----------------------------
def sky_detect(
    image_pattern,
    checkpoint_path,
    output_dir=None,
    device="cpu",
    resize_factor=6,
    threshold=0.5
):
    model, device = load_model(checkpoint_path, device)

    os.makedirs(output_dir, exist_ok=True) if output_dir else None

    print("glob.glob(image_pattern) ", glob.glob(image_pattern))
    import pdb; pdb.set_trace()
    for img_path in glob.glob(image_pattern):
        start = time.time()

        img, tensor, size = preprocess_image(img_path)

        pred = predict_mask(model, tensor, size, device)

        mask = refine_mask(img, pred, threshold=threshold)

        # Output path
        base = os.path.splitext(os.path.basename(img_path))[0]
        out_path = (
            os.path.join(output_dir, f"{base}_mask.png")
            if output_dir else f"{img_path[:-4]}_mask.png"
        )

        cv2.imwrite(out_path, mask)

        print(f"[✓] {out_path} ({time.time() - start:.2f}s)")


# -----------------------------
# Example usage
# -----------------------------

if __name__ == "__main__":
    
    parser = argparse.ArgumentParser(description="Paths")
    parser.add_argument("--image_path", type=str,  default="data/*.png", required=True, help="Images path")
    parser.add_argument("--checkpoint_path", type=str, default=r"checkpoints_G_coord_resnet50\best_ckpt.pt", required=True, help="checkpoint path")
    parser.add_argument("--output_dir", type=str, default="None", help="Output folder path")

    args = parser.parse_args()


    sky_detect(
        image_pattern = args.image_path,
        checkpoint_path = args.checkpoint_path,
        output_dir = args.output_dir,
        device="cuda" if torch.cuda.is_available() else "cpu"
    )

# python skydetect2.py --image_path "C:\Users\chahi\Desktop\HEIG\Recherche\panoramique\pfe\data\tel\*.jpg" --checkpoint_path "C:\Users\chahi\Desktop\INSA\G5\PFE\3_appariement\checkpoints_G_coord_resnet50\best_ckpt.pt" --output_dir "C:\Users\chahi\Desktop\HEIG\Recherche\panoramique\pfe\data\tel\"

    # sky_detect(
    #     image_pattern=r"C:\Users\chahi\Desktop\HEIG\Recherche\panoramique\pfe\data\tel/*.jpg",
    #     checkpoint_path = r"C:\Users\chahi\Desktop\INSA\G5\PFE\3_appariement\checkpoints_G_coord_resnet50\best_ckpt.pt",
    #     output_dir=r"C:\Users\chahi\Desktop\HEIG\Recherche\panoramique\pfe\data\tel/",
    #     device="cuda" if torch.cuda.is_available() else "cpu"
    # )