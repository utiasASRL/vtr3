"""Command line utilities
"""
import argparse
from subprocess import call


class ArgParser:

  def __init__(self, allow_unknown_args=True):
    self.parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    self.allow_unknown_args = allow_unknown_args

  def parse(self, args, display=False):
    known, unknown = self.parser.parse_known_args(args)
    self.known_dict = vars(known)
    self.unknown_dict = self._parse_unknown_args(unknown)
    if not self.allow_unknown_args:
      assert self.unknown_dict == {}
    if display:
      print("\nKnown arguments:")
      print("-----------------------------------")
      print("{:<30}{:<30}".format("Option", "Value"))
      for key, value in self.known_dict.items():
        print("{:<30}{:<30}".format(str(key), str(value)))
      print("\nUnknown arguments:")
      print("-----------------------------------")
      print("{:<30}{:<30}".format("Option", "Value"))
      for key, value in self.unknown_dict.items():
        print("{:<30}{:<30}".format(str(key), str(value)))
      print("===================================")

  def get_dict(self):
    args = {}
    args.update(self.known_dict)
    if self.allow_unknown_args:
      args["unknown_params"] = self.unknown_dict
    return args

  def _parse_unknown_args(self, args):
    """
        Parse arguments not consumed by arg parser into a dicitonary
        """
    retval = {}
    preceded_by_key = False
    for arg in args:
      if arg.startswith("--"):
        if "=" in arg:
          key = arg.split("=")[0][2:]
          value = arg.split("=")[1]
          retval[key] = value
        else:
          key = arg[2:]
          preceded_by_key = True
      elif preceded_by_key:
        retval[key] = arg
        preceded_by_key = False
    return retval


if __name__ == "__main__":
  # Argument parser check
  arg_parser = ArgParser()
  arg_parser.parser.add_argument("--test", type=int, default=0)
  arg_parser.parse(["--check", "a", "--test=2"])
  print(arg_parser.get_dict())
