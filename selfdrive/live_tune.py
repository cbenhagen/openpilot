import json


class LiveTune:
  # TODO: use dataclass and dataclasses-json when migrated to Python 3
  def __init__(self, CP=None, enabled=False, kpBP=None, kpV=None, kiBP=None, kiV=None, kf=None,
               file_path='/data/live_tune.json'):
    self._file_path = file_path
    self.enabled = enabled

    # Use defaults from CP
    self.kpBP = kpBP or list(CP.lateralTuning.pid.kpBP) if CP else [0.]
    self.kpV = kpV or list(CP.lateralTuning.pid.kpV) if CP else [0.]
    self.kiBP = kiBP or list(CP.lateralTuning.pid.kiBP) if CP else [0.]
    self.kiV = kiV or list(CP.lateralTuning.pid.kiV) if CP else [0.]
    self.kf = kf or float(CP.lateralTuning.pid.kf) if CP else 0.
    assert len(self.kpBP) == len(self.kpV)
    assert len(self.kiBP) == len(self.kiV)

    # Overwrite defaults from config file
    self.load()
    self.save()

  def load(self):
    try:
      with open(self._file_path, "r") as f:
        file_data = json.load(f)
        for key, value in file_data.items():
          if key in self.__dict__:
            self.__dict__[key] = value

    except (IOError, ValueError):
      pass

  def save(self):
    with open(self._file_path, "w") as f:
      f.write(json.dumps({k: v for k, v in self.__dict__.items() if not k.startswith('_')}, sort_keys=True, indent=4))

  def __repr__(self):
    return "enabled: {}, kiBP: {}, kiV: {}, kpBP: {}, kpV: {}".format(
      self.enabled, self.kiBP, self.kiV, self.kpBP, self.kpV
    )
