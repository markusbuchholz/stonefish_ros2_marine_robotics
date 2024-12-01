#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import openmeteo_requests
import requests_cache
import pandas as pd
from retry_requests import retry

class WeatherNode(Node):
    def __init__(self):
        super().__init__('weather_node')
        self.get_logger().info('WeatherNode has been started.')

        # Setup the Open-Meteo API client with cache and retry on error
        cache_session = requests_cache.CachedSession('.cache', expire_after=3600)
        retry_session = retry(cache_session, retries=5, backoff_factor=0.2)
        self.openmeteo = openmeteo_requests.Client(session=retry_session)

        # Parameters for the API request
        self.url = "https://marine-api.open-meteo.com/v1/marine"
        self.params = {
            "latitude": 54.544587,
            "longitude": 10.227487,
            "hourly": "wave_direction,wave_height"
        }
        
        self.timer = self.create_timer(3600, self.fetch_weather_data)
        self.fetch_weather_data()

    def fetch_weather_data(self):
        responses = self.openmeteo.weather_api(self.url, params=self.params)

        # Process the response for the first location
        response = responses[0]
        self.get_logger().info(f"Coordinates {response.Latitude()}°N {response.Longitude()}°E")
        self.get_logger().info(f"Elevation {response.Elevation()} m asl")
        self.get_logger().info(f"Timezone {response.Timezone()} {response.TimezoneAbbreviation()}")
        self.get_logger().info(f"Timezone difference to GMT+0 {response.UtcOffsetSeconds()} s")

        # Process hourly data
        hourly = response.Hourly()
        hourly_wave_height = hourly.Variables(1).ValuesAsNumpy()

        hourly_data = {
            "date": pd.date_range(
                start=pd.to_datetime(hourly.Time(), unit="s", utc=True),
                end=pd.to_datetime(hourly.TimeEnd(), unit="s", utc=True),
                freq=pd.Timedelta(seconds=hourly.Interval()),
                inclusive="left"
            )
        }
        hourly_data["wave_height"] = hourly_wave_height

        hourly_dataframe = pd.DataFrame(data=hourly_data)
        self.get_logger().info(f"\n{hourly_dataframe}")

def main(args=None):
    rclpy.init(args=args)
    node = WeatherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

