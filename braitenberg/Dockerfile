
ARG DOCKER_REGISTRY
ARG ARCH=amd64
ARG MAJOR=daffy
ARG BASE_TAG=${MAJOR}-${ARCH}

FROM ${DOCKER_REGISTRY}/duckietown/challenge-aido_lf-baseline-duckietown:${BASE_TAG}

ARG PIP_INDEX_URL="https://pypi.org/simple"
ENV PIP_INDEX_URL=${PIP_INDEX_URL}
WORKDIR /agent

RUN pip3 install -U "pip>=20.2"
COPY requirements.* ./
RUN cat requirements.* > .requirements.txt
RUN echo PIP_INDEX_URL=$PIP_INDEX_URL
RUN pip3 install --use-feature=2020-resolver -r .requirements.txt

#RUN mkdir /code/exercise_ws

COPY solution /agent/solution

CMD ["python3",  "/agent/solution/agent.py"]
