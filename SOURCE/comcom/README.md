## Installation
```
# gcloud CLI (sudo mode)
$ echo "deb [signed-by=/usr/share/keyrings/cloud.google.gpg] https://packages.cloud.google.com/apt cloud-sdk main" | sudo tee -a /etc/apt/sources.list.d/google-cloud-sdk.list

$ curl https://packages.cloud.google.com/apt/doc/apt-key.gpg | sudo apt-key --keyring /usr/share/keyrings/cloud.google.gpg add -

$ sudo apt-get update
$ sudo apt-get install google-cloud-cli
```


# Copy file to GIST repository
```
# init with gcloud
$ gcloud auth activate-service-account --key-file gist_key.json

# upload model store
$ gsutil -m cp -r model_store gs://lg-gov-gist-storage/test/model_store

# upload data-store
$ gsutil -m cp -r data_store gs://lg-gov-gist-storage/test/data_store
```


# Docker Registry Upload
```
# Registry에 대한 요청을 인증하도록 docker를 구성 -> robot registry : "asia-northeast1-docker.pkg.dev/lg-robot-dev/lg-ai-registry"
$ gcloud auth configure-docker asia-northeast1-docker.pkg.dev

# Robot 레지스트리 인증(robot registry 키로 로그인).
$ gcloud auth activate-service-account --key-file=gist_key.json

# tag 변경하기
$ sudo docker tag {DOCKER IMAGE NAME} asia-northeast1-docker.pkg.dev/lg-robot-dev/lg-ai-registry/{DOCKER IMAGE NAME}

# 이미지 push 하기
$ sudo docker push asia-northeast1-docker.pkg.dev/lg-robot-dev/lg-ai-registry/{DOCKER IMAGE NAME}

# 레지스트리에서 테스트 이미지를 pull 하기
$ sudo docker pull asia-northeast1-docker.pkg.dev/lg-robot-dev/lg-ai-registry/{DOCKER IMAGE NAME}
```
