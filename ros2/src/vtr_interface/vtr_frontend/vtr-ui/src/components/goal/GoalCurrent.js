import clsx from "clsx";
import React from "react";

import Box from "@material-ui/core/Box";
import Button from "@material-ui/core/Button";
import Card from "@material-ui/core/Card";
import CallMergeIcon from "@material-ui/icons/CallMerge";
import CheckIcon from "@material-ui/icons/Check";
import ClearIcon from "@material-ui/icons/Clear";
import MyLocationIcon from "@material-ui/icons/MyLocation";
import Typography from "@material-ui/core/Typography";
import { withStyles } from "@material-ui/core/styles";

/** @brief Parses repeat path and generate a user readable string. */
function parseGoalPath(goalPath) {
  let s = "";
  goalPath.forEach((v) => {
    let vl = parseInt(v % Math.pow(2, 32));
    let vh = parseInt((v - vl) / Math.pow(2, 32));
    s += vh.toString() + "-" + vl.toString() + ", ";
  });
  s = s.slice(0, s.length - 2);
  return s;
}

const styles = (theme) => ({});

class GoalCurrent extends React.Component {
  componentWillUnmount() {
    this.props.selectTool(null);
  }

  render() {
    const {
      active,
      className,
      goal,
      handleClick,
      removeGoal,
      requireConf,
      selectTool,
      toolsState,
    } = this.props;
    return (
      <Card className={clsx(className)}>
        <Box width={1} display={"flex"} flexDirection={"column"}>
          <Box width={1} display={"flex"} flexDirection={"row"}>
            <Box width={200} mx={1} my={"auto"}>
              <Typography variant="button">{goal.target}</Typography>
            </Box>
            <Box width={100} m={1}>
              <Button
                color={"primary"}
                disableElevation={true}
                fullWidth={true}
                size="small"
                startIcon={<ClearIcon />}
                variant={"contained"}
                onClick={(e) => removeGoal(goal, e)}
              >
                Cancel
              </Button>
            </Box>
          </Box>
          <Box display={"flex"} width={1} mx={1}>
            <Box display={"flex"} width={0.5} mr={0.5}>
              <Typography variant="button">
                {"Before: " + goal.pauseBefore.toFixed(1) + "s"}
              </Typography>
            </Box>
            <Box display={"flex"} width={0.5} ml={0.5}>
              <Typography variant="button">
                {"After: " + goal.pauseAfter.toFixed(1) + "s"}
              </Typography>
            </Box>
          </Box>
          {goal.target === "Repeat" && (
            <Box
              mx={1}
              my={"auto"}
              width={1}
              display={"flex"}
              flexDirection={"row"}
            >
              <Box
                my={"auto"}
                mr={1}
                width={0.7}
                style={{ overflowX: "scroll" }}
              >
                <Typography variant="button">
                  {"Path: " + parseGoalPath(goal.path)}
                </Typography>
              </Box>
              <Box width={0.3} ml={"auto"} my={1} mr={1}>
                <Button
                  color={active ? "secondary" : "primary"}
                  disableElevation={true}
                  size={"small"}
                  onClick={(e) => handleClick()}
                >
                  {active ? "Clear" : "Show"}
                </Button>
              </Box>
            </Box>
          )}
          <Box mx={1} mb={1} width={1} display={"flex"} flexDirection={"row"}>
            <Box width={150}>
              {goal.target === "Teach" && (
                <Button
                  color={toolsState.merge ? "secondary" : "primary"}
                  disableElevation={true}
                  fullWidth={true}
                  size="small"
                  startIcon={<CallMergeIcon />}
                  variant={"contained"}
                  onClick={() => selectTool("merge")}
                >
                  Merge
                </Button>
              )}
              {goal.target === "Repeat" && (
                <Button
                  color={toolsState.relocalize ? "secondary" : "primary"}
                  disableElevation={true}
                  fullWidth={true}
                  size="small"
                  startIcon={<MyLocationIcon />}
                  variant={"contained"}
                  onClick={() => selectTool("relocalize")}
                >
                  Relocalize
                </Button>
              )}
            </Box>
            <Box width={100} ml="auto" mr={2}>
              {(toolsState.merge || toolsState.relocalize) && (
                <Button
                  color={"secondary"}
                  fullWidth={true}
                  disableElevation={true}
                  size="small"
                  startIcon={<CheckIcon />}
                  variant={"contained"}
                  onClick={() => requireConf()}
                >
                  Confirm
                </Button>
              )}
            </Box>
          </Box>
        </Box>
      </Card>
    );
  }
}

export default withStyles(styles)(GoalCurrent);
