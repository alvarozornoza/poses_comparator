# poses_comparator

Álvaro Serra Gómez & Álvaro Zornoza Uña. UPC (2018)

Human pose estimation: RGB-D cameras vs Deep-Learning algorithm

## Run

### open-pose 
Original code at https://github.com/CMU-Perceptual-Computing-Lab/openpose and https://github.com/ildoonet/tf-pose-estimation

```bash
cd tf-pose-estimation/src
python3 run_directory_json.py --model=mobilenet_thin --folder=../../dataset/
```
### ground_truth
Original code at https://team.inria.fr/larsen/software/datasets/

```bash
cd ground_truth_estimator/build
./skViewer ../../dataset/
```
### comparator

```bash
cd comparator
python2 comparator.py
```
