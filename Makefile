test:
	branch=$(shell git rev-parse --abbrev-ref HEAD)
	repo=$(shell basename -s .git `git config --get remote.origin.url`)
	tag=selcukercan/$(repo):$(branch)
out:
	labels=$(shell ./labels.py)
build:
	docker build $(labels) -t $(tag) .
push:
	docker push $(tag)
build-no-cache:
	docker build $(labels) --no-cache -t $(tag) .
