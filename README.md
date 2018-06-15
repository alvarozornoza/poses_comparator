# poses_comparator

Álvaro Serra Gómez & Álvaro Zornoza Uña. UPC (2018)

Human pose estimation: RGB-D cameras vs Deep-Learning algorithm

## Run

### open-pose

```bash
cd tf-pose-estimation/src
python3 run_directory_json.py --model=mobilenet_thin --folder=../../dataset/
```
### ground_truth

```bash
cd ground_truth_estimator/build
./skViewer ../../dataset/
```
### comparator

```bash
cd comparator
python2 comparator.py
```