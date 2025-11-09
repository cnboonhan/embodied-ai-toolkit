# docker build . -t embodied-ai-toolkit
# docker rm embodied-ai-toolkit -f; docker run -d --network=host --name embodied-ai-toolkit --env="DISPLAY=$DISPLAY" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" embodied-ai-toolkit; docker exec -it embodied-ai-toolkit tmux

FROM huggingface/lerobot-gpu:latest

USER 0
RUN apt update && apt install tmux -y
USER 1000

WORKDIR /app
COPY . .
COPY --from=ghcr.io/astral-sh/uv:0.9.8 /uv /uvx /bin/
COPY --from=fullstorydev/grpcurl:latest /bin/grpcurl /bin/

RUN uv venv -p 3.12
RUN . .venv/bin/activate && uv sync
ENV PYTHONPATH=/app/embodied_lerobot
CMD ["rerun", "--serve-web"]