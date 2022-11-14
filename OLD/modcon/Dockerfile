ARG DOCKER_REGISTRY=docker.io
ARG ARCH=amd64
ARG MAJOR=daffy
ARG BASE_TAG=${MAJOR}-${ARCH}


FROM ${DOCKER_REGISTRY}/duckietown/challenge-aido_lf-baseline-duckietown:${BASE_TAG}


COPY requirements.* ./
RUN cat requirements.* > .requirements.txt
ARG PIP_INDEX_URL="https://pypi.org/simple"
ENV PIP_INDEX_URL=${PIP_INDEX_URL}
RUN python3 -m pip install  -r .requirements.txt

RUN python3 -m pip list

COPY solution /code/solution
COPY launchers /code

CMD ["bash", "/code/submit.sh"]
