from __future__ import annotations

import argparse
import os
from pathlib import Path

import requests

from .stub_generator import CarlaStubGenerator

PROJECT_ROOT = Path(__file__).parent.parent


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--version",
        help="The version of the CARLA Python API to generate the stubs for.",
        type=str,
    )
    parser.add_argument(
        "--output",
        help="The output directory.",
        type=str,
        default=PROJECT_ROOT / "stubs",
    )
    args = parser.parse_args()

    version = args.version
    if not version:
        # get the latest version from repository tags
        tags = requests.get(
            "https://api.github.com/repos/carla-simulator/carla/tags"
        ).json()
        version = tags[0]["name"]

    # list all files in the CARLA Python API docs repository
    carla_repo_url = f"https://api.github.com/repos/carla-simulator/carla/contents/PythonAPI/docs?ref={version}"
    response = requests.get(carla_repo_url)

    if response.status_code == 404:
        raise Exception(f"CARLA PythonAPI version {version} not found")

    data = response.json()

    # get download urls for all yaml files in the CARLA Python API docs
    file_urls = [
        file["download_url"]
        for file in data
        if file["name"].endswith((".yml", ".yaml"))
    ]

    output_path = Path(args.output) / Path(version)
    os.makedirs(output_path, exist_ok=True)

    # remove old stubs
    for file in os.listdir(output_path):
        os.remove(os.path.join(output_path, file))

    # generate stubs for all yaml files
    print(f"Generating stubs for CARLA PythonAPI version {version}... \n")

    for file_url in file_urls:
        print(f"Generating stubs for {file_url.split('/')[-1]}")
        file = requests.get(file_url).text
        stub_generator = CarlaStubGenerator(file)
        stub_generator.save_to_file(output_path)

    print(f"\nSaved stubs to {output_path.relative_to(PROJECT_ROOT)}")
