# Troubleshooting Guide

Common problems and solutions when running OpenVisionABU26.

------------------------------------------------------------------------

## Docker command not found

Error:

``` text
docker: command not found
```

Fix:

``` bash
sudo apt install docker.io
```

------------------------------------------------------------------------

## GPU not detected

Error:

``` text
could not select device driver "" with capabilities: [[gpu]]
```

Fix:

``` bash
sudo apt install nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
```

------------------------------------------------------------------------

## Camera not detected

Error:

``` text
Cannot open device: /dev/video0
```

Fix:

``` bash
ls /dev/video0
```

Then run container with:

``` bash
--device=/dev/video0
```

------------------------------------------------------------------------

## Model path not found

Error:

``` text
Failed to load model
```

Fix:

Check YAML configuration path.
