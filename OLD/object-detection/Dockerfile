ARG DOCKER_REGISTRY=docker.io
ARG ARCH=amd64
ARG MAJOR=daffy
ARG BASE_TAG=${MAJOR}-${ARCH}


FROM ${DOCKER_REGISTRY}/duckietown/challenge-aido_lf-baseline-duckietown-ml:${BASE_TAG}

COPY requirements.* ./
RUN cat requirements.* > .requirements.txt
ARG PIP_INDEX_URL="https://pypi.org/simple"
ENV PIP_INDEX_URL=${PIP_INDEX_URL}
RUN python3 -m pip install  -r .requirements.txt

RUN python3 -m pip list

COPY solution /code/solution
COPY launchers /code

RUN ls /code/solution/nn_models

RUN python3 -c "import torch; torch.hub.set_dir('/code/solution/nn_models'); model = torch.hub.load('ultralytics/yolov5', 'custom', path='/code/solution/nn_models/yolov5.pt')"

CMD ["bash", "/code/submit.sh"]
