This guide provides detailed instructions on setting up your conda environment, installing necessary packages, creating a Jupyter kernel, and downloading files from Google Drive.

## Table of Contents
1. [Conda Environment Setup](#conda-environment-setup)
2. [Conda Environment for GPU](#conda-environment-for-gpu)
3. [Creating Jupyter Kernel from Conda Environment](#creating-jupyter-kernel-from-conda-environment)
4. [Download Files from Google Drive](#download-files-from-google-drive)

## Conda Environment Setup

### Install Anaconda
1. Download Anaconda installer:
    ```bash
    wget https://repo.anaconda.com/archive/Anaconda3-2023.03-1-Linux-x86_64.sh
    ```

2. Install Anaconda:
    ```bash
    bash Anaconda3-2023.03-1-Linux-x86_64.sh
    ```

3. Initialize Anaconda:
    ```bash
    source ~/.bashrc
    ```

4. Verify Anaconda installation:
    ```bash
    conda --version
    ```

### Create and Activate a New Environment
1. Create a new environment:
    ```bash
    conda create --name myenv python=3.8
    ```

2. Activate the new environment:
    ```bash
    conda activate myenv
    ```

### Install Packages
1. Install packages with Conda:
    ```bash
    conda install numpy pandas matplotlib
    ```

2. Install packages with pip:
    ```bash
    pip install requests beautifulsoup4
    ```

3. Upgrade pip:
    ```bash
    python -m pip install --upgrade pip
    ```

4. Verify Python installation:
    ```bash
    python --version
    ```

## Conda Environment for GPU

### Create and Activate the Environment
1. Create the environment:
    ```bash
    conda create --name ev_gpu python=3.9
    ```

2. Activate the new environment:
    ```bash
    conda activate ev_gpu
    ```

### Install CUDA and cuDNN
1. Install CUDA:
    ```bash
    conda install -c conda-forge cudatoolkit=11.2
    ```

2. Install cuDNN:
    ```bash
    conda install -c conda-forge cudnn=8.1.0
    ```

### Install TensorFlow
1. Install TensorFlow 2.10:
    ```bash
    pip install tensorflow==2.10.0
    ```

2. Verify the installation:
    ```bash
    python -c "import tensorflow as tf; print(tf.config.list_physical_devices('GPU'))"
    ```

## Creating Jupyter Kernel from Conda Environment

### Install ipykernel
1. Activate your conda environment:
    ```bash
    source activate nirob
    ```

2. Install ipykernel in your conda environment:
    ```bash
    conda install -c anaconda ipykernel
    ```

3. Add your conda environment as a Jupyter kernel:
    ```bash
    python -m ipykernel install --user --name nirob --display-name "Python (nirob)"
    ```

### Change Kernel Name
1. Change kernel name:
    ```bash
    python -m ipykernel install --user --name nirob --display-name "Python (nirob)"
    ```

## Download Files from Google Drive

### Using `gdown`
1. Download files from Google Drive:
    ```bash
    gdown https://drive.google.com/uc?id=FILE_ID
    ```

### Using `rclone`
1. Download folder from Google Drive:
    ```bash
    rclone copy "mydrive:1AaHynzBMNcE4vjLQ-roSZDIYF304BYRI"
    ```

2. Download folder using `gdown`:
    ```bash
    gdown --folder https://drive.google.com/drive/folders/1AaHynzBMNcE4vjLQ-roSZDIYF304BYRI
    ```

## Additional Notes
- If `conda init` does not work, source the Conda script manually:
    ```bash
    source /opt/conda/etc/profile.d/conda.sh
    ```
