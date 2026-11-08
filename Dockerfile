FROM python:3.12-slim-bookworm

RUN apt-get update \
 && apt-get install -y --no-install-recommends \
      gcc libc6-dev make \
      gcc-avr avr-libc binutils-avr \
      libsdl2-2.0-0 fonts-dejavu-core fontconfig \
 && rm -rf /var/lib/apt/lists/*

RUN pip install --no-cache-dir pygame==2.6.1

ENV SDL_AUDIODRIVER=dummy \
    PYGAME_HIDE_SUPPORT_PROMPT=1 \
    PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1 \
    HOME=/tmp

WORKDIR /work
CMD ["./scripts/sim.sh"]
