import os
from dotenv import load_dotenv

# Load the .env file from the project root
load_dotenv()

# Extract variables safely
REDIS_URL = os.environ.get("REDIS_URL", "")
# Backend API — route list includes routeCoordinates (used as polyline; no Google call needed)
API_ROUTES_URL = os.environ.get("API_ROUTES_URL", "")

# Optional fallback only when routeCoordinates are missing from API_ROUTES_URL
API_DIRECTIONS_BASE_URL = os.environ.get("API_DIRECTIONS_BASE_URL", "")

API_BEARER_TOKEN = os.environ.get("API_BEARER_TOKEN", "")
TOPIC_PREDICTED_OUT = os.environ.get("TOPIC_PREDICTED_OUT", "gps:predicted")

if not API_ROUTES_URL or not API_BEARER_TOKEN:
    import logging
    logger = logging.getLogger(__name__)
    logger.warning("API_ROUTES_URL or API_BEARER_TOKEN is not defined in the environment!")
