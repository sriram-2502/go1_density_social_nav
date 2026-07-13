SHELL := /bin/bash

IMAGE ?= object_detection
IMAGE_TAR ?= docker/image_backup/object_detection_jetson_agx_xavier_jp512_zed512.tar
DOCKER_SOURCE_TAR ?= docker/source_backup/docker_source.tar
DOCKER_SOURCE_DIR ?= docker/source_backup/unpacked
JETSON_BAG_DIR ?= trial_data/raw
PLOT_DIR ?= trial_data/processed
DOCS_PORT ?= 8000

.PHONY: help docker-load docker-build docker-run docker-save docker-unpack-source analyze site-preview check

help:
	@echo "Targets:"
	@echo "  make docker-load     Load the private saved Jetson image tar"
	@echo "  make docker-build    Rebuild object_detection from docker/Dockerfile"
	@echo "  make docker-unpack-source  Unpack an optional private Docker source tar"
	@echo "  make docker-run      Run the Jetson camera/perception container"
	@echo "  make analyze         Generate plots from paired ROS bags"
	@echo "  make site-preview    Preview the GitHub Pages site locally"
	@echo "  make check           Syntax-check scripts that can be checked on the host"

# Exact restoration path for the lab image. The tar is intentionally gitignored.
docker-load:
	@if [ ! -f "$(IMAGE_TAR)" ]; then \
		echo "Missing $(IMAGE_TAR)"; \
		echo "Copy the private docker save tar into docker/image_backup/ first."; \
		exit 1; \
	fi
	sudo docker load -i "$(IMAGE_TAR)"

# Reproducible source-build path. Run this on Jetson AGX Xavier / JetPack 5.1.2.
docker-build:
	sudo docker build -f docker/Dockerfile -t "$(IMAGE):latest" .

docker-run:
	IMAGE="$(IMAGE)" ./docker/run_camera_container.sh

# Optional private source/context backup. Do not commit docker/source_backup/.
docker-unpack-source:
	@if [ ! -f "$(DOCKER_SOURCE_TAR)" ]; then \
		echo "Missing $(DOCKER_SOURCE_TAR)"; \
		echo "Set DOCKER_SOURCE_TAR=/path/to/source.tar or put it there first."; \
		exit 1; \
	fi
	mkdir -p "$(DOCKER_SOURCE_DIR)"
	tar -xf "$(DOCKER_SOURCE_TAR)" -C "$(DOCKER_SOURCE_DIR)"
	@find "$(DOCKER_SOURCE_DIR)" -maxdepth 2 -type f | sort | head -50

# Optional private backup target. Do not commit docker/image_backup/.
docker-save:
	mkdir -p docker/image_backup
	sudo docker save "$(IMAGE):latest" -o "$(IMAGE_TAR)"

analyze:
	/usr/bin/python3 scripts/analyze_bag_runs.py --bag-dir "$(JETSON_BAG_DIR)" --out-dir "$(PLOT_DIR)"

site-preview:
	python3 -m http.server "$(DOCS_PORT)" -d docs

check:
	bash -n docker/run_camera_container.sh scripts/sync_jetson_bags.sh
	python3 -m py_compile scripts/analyze_bag_runs.py scripts/bag2mp4.py scripts/obsbag2csv.py scripts/posbagtocsv.py scripts/velbag2csv.py
