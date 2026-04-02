import numpy as np
import matplotlib.pyplot as plt
import os
import shutil

# Load all saved frames
frame_dir = 'frames'
files = sorted(os.listdir(frame_dir))  # frame_00000.npy, frame_00001.npy, ...

rows = []
for f in files:
    if f.endswith('.npy'):
        frame = np.load(os.path.join(frame_dir, f))
        rows.append(frame)

# Stack into 2D image — shape: (num_frames, 1024)
image = np.stack(rows, axis=0)

plt.figure(figsize=(12, 6))
plt.imshow(image, cmap='gray', aspect='auto')
plt.colorbar(label='Intensity (10-bit)')
plt.xlabel('Pixel (across)')
plt.ylabel('Frame (down)')
plt.title('EPC901 2D Reconstructed Image')
plt.show()

# Save as PNG
plt.imsave('output_image.png', image, cmap='gray')