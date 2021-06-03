import clsx from "clsx";
import shortid from "shortid";
import React from "react";

import AddIcon from "@material-ui/icons/Add";
import ArrowBackIosIcon from "@material-ui/icons/ArrowBackIos";
import ArrowForwardIosIcon from "@material-ui/icons/ArrowForwardIos";
import Box from "@material-ui/core/Box";
import Button from "@material-ui/core/Button";
import ClearIcon from "@material-ui/icons/Clear";
import Drawer from "@material-ui/core/Drawer";
import PauseIcon from "@material-ui/icons/Pause";
import PlayArrowIcon from "@material-ui/icons/PlayArrow";
import StopIcon from "@material-ui/icons/Stop";
import StorageIcon from "@material-ui/icons/Storage";
import { withStyles } from "@material-ui/core/styles";
import {
  sortableContainer,
  sortableElement,
  arrayMove,
} from "react-sortable-hoc";

import GoalCard from "./GoalCard";
import GoalCurrent from "./GoalCurrent";
import GoalForm from "./GoalForm";

const Goal = sortableElement((props) => {
  // \todo Check why this extra div adds smoother animation.
  return (
    <Box width={1}>
      <GoalCard
        active={props.active}
        goal={props.goal}
        id={props.id}
        handleClick={props.handleClick}
        removeGoal={props.removeGoal}
      ></GoalCard>
    </Box>
  );
});

const GoalContainer = sortableContainer((props) => {
  const { className, maxHeight, top } = props;
  return (
    <Box
      className={className}
      width={1}
      maxHeight={maxHeight}
      style={{ overflowY: "scroll", marginTop: top }}
    >
      {props.children}
    </Box>
  );
});

// Style
const currGoalCardHeight = 160;
const goalFormHeight = 300;
const goalPanelWidth = 300;
const topButtonHeight = 10;
const transitionDuration = 300;
const styles = (theme) => ({
  goalPanelButton: {
    transition: theme.transitions.create(["left"], {
      duration: transitionDuration,
    }),
  },
  goalContainer: {},
  goalContainerHelper: {
    zIndex: 2000, // \todo This is a magic number.
  },
});

class GoalManager extends React.Component {
  constructor(props) {
    super(props);

    this.state = {
      goalPanelOpen: false,
      addingGoal: false,
      lockGoals: true,
      goals: [], // {id, target, vertex, path, pauseBefore, pauseAfter, inProgress}
      lockStatus: true,
      status: "PAUSED",
      selectedGoalID: "",
      windowHeight: 0,
    };

    this._loadInitGoalsAndState();
  }

  componentDidMount() {
    console.debug("[GoalManager] componentDidMount: Goal manager mounted.");
    // SocketIO listeners
    this.props.socket.on("goal/new", this._newGoalCb.bind(this));
    this.props.socket.on("goal/cancelled", this._removeGoalCb.bind(this));
    this.props.socket.on("goal/error", this._removeGoalCb.bind(this));
    this.props.socket.on("goal/success", this._removeGoalCb.bind(this));
    this.props.socket.on("goal/started", this._startedGoalCb.bind(this));
    this.props.socket.on("status", this._statusCb.bind(this));
    // Auto-adjusts window height
    window.addEventListener("resize", this._updateWindowHeight.bind(this));
    this._updateWindowHeight();
  }

  componentWillUnmount() {
    // SocketIO listeners
    this.props.socket.off("goal/new", this._newGoalCb.bind(this));
    this.props.socket.off("goal/cancelled", this._removeGoalCb.bind(this));
    this.props.socket.off("goal/error", this._removeGoalCb.bind(this));
    this.props.socket.off("goal/success", this._removeGoalCb.bind(this));
    this.props.socket.off("goal/started", this._startedGoalCb.bind(this));
    this.props.socket.off("status", this._statusCb.bind(this));
    window.removeEventListener("resize", this._updateWindowHeight.bind(this));
  }

  render() {
    const {
      addingGoalPath,
      addingGoalType,
      classes,
      className,
      setAddingGoalPath,
      setAddingGoalType,
      requireConf,
      selectTool,
      toolsState,
    } = this.props;
    const {
      addingGoal,
      goals,
      goalPanelOpen,
      lockGoals,
      lockStatus,
      selectedGoalID,
      status,
      windowHeight,
    } = this.state;
    return (
      <>
        <Box
          className={classes.goalPanelButton}
          position={"absolute"}
          top={0}
          left={goalPanelOpen ? goalPanelWidth + 10 : 0}
          zIndex={1000}
          m={0.5}
          width={120}
        >
          <Button
            color={goalPanelOpen ? "secondary" : "primary"}
            disableElevation={true}
            variant={"contained"}
            fullWidth={true}
            startIcon={<StorageIcon />}
            endIcon={
              goalPanelOpen ? <ArrowBackIosIcon /> : <ArrowForwardIosIcon />
            }
            onClick={this._toggleGoalPanel.bind(this)}
          >
            Goals
          </Button>
        </Box>
        {goals.length > 0 && goals[0].inProgress && (
          <Box
            position={"absolute"}
            top={45}
            width={goalPanelWidth}
            zIndex={2000}
            ml={0.5}
          >
            <GoalCurrent
              active={goals[0].id === selectedGoalID}
              goal={goals[0]}
              handleClick={this._handleSelect.bind(this, goals[0].id)}
              removeGoal={this._removeGoal.bind(this)}
              requireConf={requireConf}
              selectTool={selectTool}
              toolsState={toolsState}
            ></GoalCurrent>
          </Box>
        )}
        <Drawer
          className={clsx(className)}
          variant="persistent"
          anchor="left"
          open={goalPanelOpen}
          transitionDuration={transitionDuration}
          PaperProps={{
            elevation: 0,
            style: {
              backgroundColor: "rgba(255, 255, 255, 0.0)",
            },
          }}
        >
          <Box
            width={goalPanelWidth}
            display={"flex"}
            justifyContent={"center"}
            flexDirection={"row"}
            m={0.5}
          >
            <Box width={100} m={0.5}>
              {status === "PAUSED" || status === "PENDING_PAUSE" ? (
                <Button
                  color={"primary"}
                  disabled={lockStatus}
                  disableElevation={true}
                  fullWidth={true}
                  startIcon={<PlayArrowIcon />}
                  size={"small"}
                  variant={"contained"}
                  onClick={this._handlePlay.bind(this)}
                >
                  Play
                </Button>
              ) : (
                <Button
                  color={"secondary"}
                  disabled={lockStatus}
                  disableElevation={true}
                  fullWidth={true}
                  startIcon={<PauseIcon />}
                  size={"small"}
                  variant={"contained"}
                  onClick={this._handlePause.bind(this)}
                >
                  Pause
                </Button>
              )}
            </Box>
            <Box width={100} m={0.5}>
              <Button
                color={"primary"}
                disabled={lockStatus}
                disableElevation={true}
                fullWidth={true}
                startIcon={<StopIcon />}
                size={"small"}
                variant={"contained"}
                onClick={this._handleClear.bind(this)}
              >
                Clear
              </Button>
            </Box>
          </Box>
          <Box
            width={goalPanelWidth}
            display={"flex"}
            justifyContent={"center"}
            flexDirection={"row"}
            m={0.5}
          >
            <GoalContainer
              className={classes.goalContainer}
              helperClass={classes.goalContainerHelper}
              disabled={lockGoals}
              distance={2}
              lockAxis="y"
              onSortEnd={(e) => {
                this._moveGoal(e.oldIndex, e.newIndex);
              }}
              useDragHandle
              // Cannot pass through className because it depends on state.
              // \todo jsx error that causes incorrect return from ?: operator?
              maxHeight={
                goals.length > 0 && goals[0].inProgress
                  ? windowHeight -
                    topButtonHeight -
                    goalFormHeight -
                    currGoalCardHeight
                  : windowHeight - topButtonHeight - goalFormHeight
              }
              // Cannot pass through className because it depends on state.
              top={
                goals.length > 0 && goals[0].inProgress ? currGoalCardHeight : 0
              }
            >
              {goals.map((goal, index) => {
                if (goal.inProgress) return null;
                return (
                  <Goal
                    key={shortid.generate()}
                    active={goal.id === selectedGoalID}
                    disabled={lockGoals}
                    goal={goal}
                    id={index}
                    index={index}
                    handleClick={this._handleSelect.bind(this, goal.id)}
                    removeGoal={this._removeGoal.bind(this)}
                  />
                );
              })}
            </GoalContainer>
          </Box>
          <Box width={goalPanelWidth} m={0.5}>
            {addingGoal && (
              <GoalForm
                goalPath={addingGoalPath}
                goalType={addingGoalType}
                setGoalPath={setAddingGoalPath}
                setGoalType={setAddingGoalType}
                submit={this._submitGoal.bind(this)}
              ></GoalForm>
            )}
          </Box>
          <Box
            width={goalPanelWidth}
            display={"flex"}
            justifyContent={"center"}
            m={0.5}
          >
            <Button
              disableElevation={true}
              color={addingGoal ? "secondary" : "primary"}
              size={"small"}
              startIcon={addingGoal ? <ClearIcon /> : <AddIcon />}
              variant={"contained"}
              onClick={this._toggleGoalForm.bind(this)}
            >
              {addingGoal ? "Cancel" : "Add A Goal"}
            </Button>
          </Box>
        </Drawer>
      </>
    );
  }

  /** Shows/hides the goal panel. */
  _toggleGoalPanel() {
    this.setState((state) => ({ goalPanelOpen: !state.goalPanelOpen }));
  }

  /** Shows/hides the goal addition form. */
  _toggleGoalForm() {
    this.setState((state) => ({
      addingGoal: !state.addingGoal,
    }));
  }

  /**
   * @brief Calls graphMap to display user specified vertices of a goal when user
   * clicks on / selects an added goal. Also sets the corresponding goal card to
   * active.
   * @param {number} id The selected goal id.
   */
  _handleSelect(id) {
    console.debug("[GoalManager] _handleSelect: id:", id);
    let selectedGoalPath = [];
    this.setState(
      (state) => {
        if (state.selectedGoalID === id) return { selectedGoalID: "" };
        for (let goal of state.goals) {
          if (goal.id === id) {
            selectedGoalPath = goal.path;
            break;
          }
        }
        return { selectedGoalID: id };
      },
      () => this.props.setSelectedGoalPath(selectedGoalPath)
    );
  }

  /**
   * @brief Fetches a complete list of goals and server status upon
   * initialization, and unlocks goals and status.
   */
  _loadInitGoalsAndState() {
    console.log("Loading initial goals and status.");
    fetch("/api/goal/all")
      .then((response) => {
        if (response.status !== 200) {
          console.error("Fetch initial goals failed:" + response.status);
          return;
        }
        // Examine the text in the response
        response.json().then((data) => {
          console.debug("[GoalManager] _loadInitGoalsAndState: data:", data);
          this.setState({
            goals: data.goals,
            lockGoals: false,
            status: data.status,
            lockStatus: false,
          });
        });
      })
      .catch((err) => {
        console.error("Fetch error:", err);
      });
  }

  /**
   * @brief Submits the user specified goal via SocketIO.
   * @param {Object} goal The goal to be submitted.
   * @param {Object} cb Resets GoalForm upon succeed.
   */
  _submitGoal(goal, cb) {
    console.debug("[GoalManager] _submitGoal: goal:", goal);
    let socketCb = (success, msg) => {
      console.debug("[GoalManager] _submitGoal: success:", success);
      if (!success) console.error(msg);
      cb(success);
      this.setState({ lockGoals: false });
    };
    let emit = true;
    this.setState(
      (state) => {
        if (state.lockGoals) {
          emit = false;
          return;
        }
        return { lockGoals: true };
      },
      () => {
        if (emit) this.props.socket.emit("goal/add", goal, socketCb.bind(this));
        else cb(false);
      }
    );
  }

  /**
   * @brief Removes the user specified goal via SocketIO.
   * @param {Object} goal The goal to be removed.
   */
  _removeGoal(goal) {
    console.debug("[GoalManager] _removeGoal: goal:", goal);
    let socketCb = (success, msg) => {
      console.debug("[GoalManager] _removeGoal: success:", success);
      if (!success) console.error(msg);
      this.setState({ lockGoals: false });
    };
    let emit = true;
    this.setState(
      (state) => {
        if (state.lockGoals) {
          emit = false;
          return;
        }
        return { lockGoals: true };
      },
      () => {
        if (emit)
          this.props.socket.emit("goal/cancel", goal, socketCb.bind(this));
      }
    );
  }

  /**
   * @brief Changes order of goals via SocketIO.
   * @param {Number} oldIdx Old index of the goal to be re-ordered.
   * @param {Number} newIdx New index of the goal to be re-ordered.
   */
  _moveGoal(oldIdx, newIdx) {
    console.debug("[GoalManager] _moveGoal: old:", oldIdx, "new:", newIdx);
    let socketCb = (success, msg) => {
      console.debug("[GoalManager] _moveGoal: success:", success);
      if (!success) console.error(msg);
      this.setState({ lockGoals: false });
    };
    let emit = true;
    let goals = null;
    this.setState(
      (state) => {
        if (state.lockGoals) {
          emit = false;
          return;
        }
        goals = state.goals.slice();
        return { lockGoals: true };
      },
      () => {
        let newGoals = arrayMove(goals, oldIdx, newIdx);
        let req = { goalID: goals[oldIdx].id };
        if (newIdx < newGoals.length - 1) {
          req.before = newGoals[newIdx + 1].id;
        }
        if (emit) this.props.socket.emit("goal/move", req, socketCb.bind(this));
      }
    );
  }

  /**
   * @brief Mission server callback on remote goal addition.
   * @param {Object} goal
   */
  _newGoalCb(goal) {
    console.debug("[GoalManager] _newGoalCb: goal:", goal);
    this.setState((state) => {
      return {
        goals: [...state.goals, goal],
      };
    });
  }

  /** Mission server callback on remote goal deletion/finish/error.
   *
   * @param {Object} goal
   */
  _removeGoalCb(goal) {
    console.debug("[GoalManager] _removeGoalCb: goal:", goal);
    let selectedGoalPath = [];
    this.setState(
      (state, props) => {
        selectedGoalPath = props.selectedGoalPath;
        for (var i = 0; i < state.goals.length; i++) {
          if (state.goals[i].id === goal.id) {
            state.goals.splice(i, 1);
            if (state.selectedGoalID === goal.id) {
              selectedGoalPath = [];
              return { goals: state.goals, selectedGoalID: "" };
            } else return { goals: state.goals };
          }
        }
      },
      () => this.props.setSelectedGoalPath(selectedGoalPath)
    );
  }

  /** Mission server callback on remote goal start.
   *
   * @param {Object} goal
   */
  _startedGoalCb(goal) {
    console.debug("[GoalManager] _startedGoalCb: goal:", goal);
    this.setState((state) => {
      for (let i = 0; i < state.goals.length; i++) {
        if (state.goals[i].id === goal.id) {
          state.goals[i].inProgress = true;
          break;
        }
      }
      return { goals: state.goals };
    });
  }

  /** Mission server callback when a new status message is received
   *
   * This is also called after reordering goals to update goal order.
   * \todo Separate status update and goal order update?
   * \todo Check if it is possible that the returned goal length is different.
   * @param {Object} data {queue, state} where queue contains goal ids.
   */
  _statusCb(data) {
    console.debug("[GoalManager] _statusCb: data:", data);

    this.setState((state) => {
      state.goals.sort(
        (a, b) => data.queue.indexOf(a.id) - data.queue.indexOf(b.id)
      );
      return { status: data.state, goals: state.goals };
    });
  }

  /** Asks the server to un-pause via SocketIO. */
  _handlePlay() {
    console.debug("[GoalManager] _handlePlay");
    let socketCb = (success, msg) => {
      console.debug("[GoalManager] _handlePlay: success:", success);
      if (!success) console.error(msg);
      this.setState({ lockStatus: false });
    };
    let emit = true;
    this.setState(
      (state) => {
        if (state.lockStatus) {
          emit = false;
          return;
        }
        return { lockStatus: true };
      },
      () => {
        if (emit)
          this.props.socket.emit(
            "pause",
            { paused: false },
            socketCb.bind(this)
          );
      }
    );
  }

  /** Asks the server to pause via SocketIO. */
  _handlePause() {
    console.debug("[GoalManager] _handlePause");
    let socketCb = (success, msg) => {
      console.debug("[GoalManager] _handlePause: success:", success);
      if (!success) console.error(msg);
      this.setState({ lockStatus: false });
    };
    let emit = true;
    this.setState(
      (state) => {
        if (state.lockStatus) {
          emit = false;
          return;
        }
        return { lockStatus: true };
      },
      () => {
        if (emit)
          this.props.socket.emit(
            "pause",
            { paused: true },
            socketCb.bind(this)
          );
      }
    );
  }

  /** Asks the server to pause and then cancel all goals via SocketIO. */
  _handleClear() {
    console.debug("[GoalManager] _handleClear");
    let socketGoalsCb = (success, msg) => {
      console.debug("[GoalManager] _handleClear: goals success:", success);
      if (!success) console.error(msg);
      this.setState({ lockGoals: false });
    };
    let socketStatusCb = (success, msg) => {
      console.debug("[GoalManager] _handleClear: status success:", success);
      if (!success) console.error(msg);
      this.setState({ lockStatus: false }, () =>
        this.props.socket.emit("goal/cancel/all", socketGoalsCb.bind(this))
      );
    };
    let emit = true;
    this.setState(
      (state) => {
        if (state.lockStatus || state.lockGoals) {
          emit = false;
          return;
        }
        return { lockStatus: true, lockGoals: true };
      },
      () => {
        if (emit)
          this.props.socket.emit(
            "pause",
            { paused: true },
            socketStatusCb.bind(this)
          );
      }
    );
  }

  /** Updates internal windowHeight state variable to track inner height of the
   * current window.
   */
  _updateWindowHeight() {
    this.setState({ windowHeight: window.innerHeight });
  }
}

export default withStyles(styles)(GoalManager);
