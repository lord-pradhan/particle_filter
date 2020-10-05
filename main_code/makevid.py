import os

os.system("ffmpeg -r 90 -i images/%d.jpg -vcodec mpeg4 -y log1.mp4")