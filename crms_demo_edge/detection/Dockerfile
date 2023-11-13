FROM hogili89/cloud:base-det
COPY ./ /home/detection

RUN pip install Flask tqdm scipy
RUN pip install scikit-image

# build 
WORKDIR /home/detection/utils
RUN python setup.py build develop

# python path
WORKDIR /home/detection

ENTRYPOINT [ "python" ]
CMD [ "main.py" ]
