FROM duckietown/challenge-aido_lf-baseline-duckietown-ml:daffy-amd64

RUN pip3 install -U "pip>=20.2"
COPY requirements.* ./
RUN cat requirements.* > .requirements.txt
RUN  pip3 install --use-feature=2020-resolver -r .requirements.txt

RUN pip list

RUN mkdir /code/solution

COPY solution /code/solution
COPY launchers /code

RUN ls /code/solution/nn_models

RUN python3 -c "import torch; torch.hub.set_dir('/code/solution/nn_models'); model = torch.hub.load('ultralytics/yolov5', 'custom', path='/code/solution/nn_models/yolov5.pt')"

CMD ["bash", "/code/submit.sh"]