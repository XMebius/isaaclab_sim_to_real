from deploy.utils.runner import Runner
import yaml

# main
if __name__ == "__main__":
    # import configuration
    cfg_path = "config.yaml" # TODO: check the real path
    with open(cfg_path, "r") as f:
        config = yaml.safe_load(f)

    # create the runner
    runner = Runner(config)
    runner.run()
       