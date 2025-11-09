# docker build . -t embodied-ai-toolkit
# docker run -it --network=host --rm embodied-ai-toolkit tmux

FROM huggingface/lerobot-gpu:latest

USER 0
RUN apt update && apt install tmux -y
USER 1000

WORKDIR /app
COPY . .
COPY --from=ghcr.io/astral-sh/uv:0.9.8 /uv /uvx /bin/

RUN uv venv -p 3.12
RUN . .venv/bin/activate && uv sync
CMD ["sleep", "infinity"]