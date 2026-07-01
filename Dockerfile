# RouteEye Motion Prediction Layer
# Single-threaded Redis GPS prediction daemon

FROM python:3.11-slim

# Prevent Python from writing .pyc files and enable unbuffered output
ENV PYTHONDONTWRITEBYTECODE=1
ENV PYTHONUNBUFFERED=1

WORKDIR /app

# Install dependencies first (layer caching)
COPY requirements.txt .
RUN pip install --no-cache-dir redis && \
    pip install --no-cache-dir -r requirements.txt

# Copy application code
COPY pyproject.toml .
COPY main.py .
COPY motion_prediction/ motion_prediction/

# Install the package in editable mode
RUN pip install --no-cache-dir -e .

# Run the prediction daemon
CMD ["python", "main.py"]
