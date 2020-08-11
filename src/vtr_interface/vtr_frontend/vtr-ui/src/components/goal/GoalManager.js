import React from "react";
import { withStyles } from "@material-ui/core/styles";
import Button from "@material-ui/core/Button";
import IconButton from "@material-ui/core/IconButton";
import Drawer from "@material-ui/core/Drawer";
import clsx from "clsx";
import shortid from "shortid";

import {
  sortableContainer,
  sortableElement,
  arrayMove,
} from "react-sortable-hoc";

import GoalCard from "./GoalCard";
import GoalCurrent from "./GoalCurrent";
import GoalForm from "./GoalForm";

const Goal = sortableElement((props) => {
  // \todo check: this div adds smoother animation.
  return (
    <div>
      <GoalCard
        id={props.id}
        active={props.active}
        goal={props.goal}
        removeGoal={props.removeGoal}
        onClick={props.onClick}
      ></GoalCard>
    </div>
  );
});

const GoalContainer = sortableContainer((props) => {
  const { className, maxHeight, top } = props;
  return (
    <div
      className={className}
      style={{ overflowY: "scroll", maxHeight: maxHeight, marginTop: top }}
    >
      {props.children}
    </div>
  );
});

// Style
const minGap = 5;
const topButtonHeight = 15;
const currGoalCardHeight = 200;
const goalFormHeight = 300;
const goalPanelButtonHeight = 50;
const goalPanelWidth = 300;
const styles = (theme) => ({
  goalPanelButton: {
    position: "absolute",
    width: 100,
    height: goalPanelButtonHeight,
    backgroundColor: "rgba(255, 255, 255, 0.7)",
    "&:hover": {
      backgroundColor: "rgba(255, 255, 255, 0.7)",
    },
    zIndex: 1000, // \todo This is a magic number.
    top: minGap,
    left: minGap,
    transition: theme.transitions.create(["left", "width"], {
      duration: theme.transitions.duration.leavingScreen,
    }),
  },
  goalPanelButtonShift: {
    left: goalPanelWidth + minGap,
    transition: theme.transitions.create(["left", "width"], {
      duration: theme.transitions.duration.enteringScreen,
    }),
  },
  goalPanel: (props) => ({
    flexShrink: 100,
    width: goalPanelWidth,
  }),
  goalPanelPaper: {
    backgroundColor: "rgba(255, 255, 255, 0.2)",
    width: goalPanelWidth,
  },
  goalCurrent: {
    position: "absolute",
    top: goalPanelButtonHeight + 2 * minGap,
    width: goalPanelWidth,
    zIndex: 2000,
  },
  goalContainer: {},
  goalContainerHelper: {
    zIndex: 2000, // \todo This is a magic number.
  },
  goalButton: {
    marginLeft: "auto",
    marginRight: "auto",
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
        <IconButton
          className={clsx(classes.goalPanelButton, {
            [classes.goalPanelButtonShift]: goalPanelOpen,
          })}
          onClick={this._toggleGoalPanel.bind(this)}
          // color="inherit"
          // aria-label="open drawer"
          // edge="start"
        >
          Goal Panel
        </IconButton>
        {goals.length > 0 && goals[0].inProgress && (
          <>
            <Button className={classes.goalCurrent} />
            <GoalCurrent
              className={classes.goalCurrent}
              active={goals[0].id === selectedGoalID}
              goal={goals[0]}
              removeGoal={this._removeGoal.bind(this)}
              onClick={this._handleSelect.bind(this, goals[0].id)}
            ></GoalCurrent>
          </>
        )}
        <Drawer
          className={clsx(classes.goalPanel, className)}
          variant="persistent"
          anchor="left"
          open={goalPanelOpen}
          classes={{
            paper: clsx(classes.goalPanelPaper, className),
          }}
        >
          <div style={{ marginLeft: "auto", marginRight: "auto" }}>
            {status === "PAUSED" || status === "PENDING_PAUSE" ? (
              <Button
                disabled={lockStatus}
                onClick={this._handlePlay.bind(this)}
              >
                Play
              </Button>
            ) : (
              <Button
                disabled={lockStatus}
                onClick={this._handlePause.bind(this)}
              >
                Pause
              </Button>
            )}
            <Button
              disabled={lockStatus}
              onClick={this._handleClear.bind(this)}
            >
              Clear
            </Button>
          </div>
          <GoalContainer
            className={classes.goalContainer}
            helperClass={classes.goalContainerHelper}
            disabled={lockGoals}
            distance={2}
            lockAxis="y"
            useDragHandle
            onSortEnd={(e) => {
              this._moveGoal(e.oldIndex, e.newIndex);
            }}
            // cannot pass through className because it depends on state.
            // jsx error that causes incorrect return from ?: operator?
            maxHeight={
              goals.length > 0 && goals[0].inProgress
                ? windowHeight -
                  topButtonHeight -
                  goalFormHeight -
                  4 * minGap -
                  currGoalCardHeight -
                  minGap
                : windowHeight - topButtonHeight - goalFormHeight - 4 * minGap
            }
            top={
              goals.length > 0 && goals[0].inProgress
                ? topButtonHeight + 2 * minGap + currGoalCardHeight + minGap
                : topButtonHeight + 2 * minGap
            }
          >
            {goals.map((goal, index) => {
              if (goal.inProgress) return null;
              return (
                <Goal
                  key={shortid.generate()}
                  active={goal.id === selectedGoalID}
                  index={index}
                  goal={goal}
                  id={index}
                  removeGoal={this._removeGoal.bind(this)}
                  disabled={lockGoals}
                  onClick={this._handleSelect.bind(this, goal.id)}
                />
              );
            })}
          </GoalContainer>
          {addingGoal && (
            <GoalForm
              goalPath={addingGoalPath}
              goalType={addingGoalType}
              setGoalPath={setAddingGoalPath}
              setGoalType={setAddingGoalType}
              submit={this._submitGoal.bind(this)}
            ></GoalForm>
          )}
          <IconButton
            className={classes.goalButton}
            color="inherit"
            aria-label="add goal"
            edge="start"
            onClick={this._toggleGoalForm.bind(this)}
          >
            {addingGoal ? "Cancel" : "Add Goal"}
          </IconButton>
        </Drawer>
      </>
    );
  }

  /** Shows/hides the goal panel. */
  _toggleGoalPanel() {
    this.setState((state) => ({ goalPanelOpen: !state.goalPanelOpen }));
  }

  /** Shows/hides the goal addition form */
  _toggleGoalForm() {
    this.setState((state) => ({
      addingGoal: !state.addingGoal,
    }));
  }

  _handleSelect(id) {
    console.log("[GoalManager] _handleSelect: id:", id);
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

  /** Fetches a complete list of goals and server status upon initialization,
   * and unlocks goals and status.
   */
  _loadInitGoalsAndState() {
    console.log("Loading initial goals and status.");
    fetch("/api/goal/all")
      .then((response) => {
        if (response.status !== 200) {
          console.log("Fetch initial goals failed: " + response.status);
          return;
        }
        // Examine the text in the response
        response.json().then((data) => {
          console.log("[GoalManager] _loadInitGoalsAndState: data:", data);
          this.setState({
            goals: data.goals,
            lockGoals: false,
            status: data.status,
            lockStatus: false,
          });
        });
      })
      .catch((err) => {
        console.log("Fetch error: ", err);
      });
  }

  /** Submits the user specified goal via SocketIO.
   *
   * @param {Object} goal The goal to be submitted.
   * @param {Object} cb Resets GoalForm upon succeed.
   */
  _submitGoal(goal, cb) {
    console.log("[GoalManager] _submitGoal: goal:", goal);
    let socketCb = (success, msg) => {
      console.log("[GoalManager] _submitGoal: success:", success, "msg:", msg);
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

  /** Removes the user specified goal via SocketIO.
   *
   * @param {Object} goal The goal to be removed.
   */
  _removeGoal(goal) {
    console.log("[GoalManager] _removeGoal: goal:", goal);
    let socketCb = (success, msg) => {
      console.log("[GoalManager] _removeGoal: success:", success, "msg:", msg);
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

  /** Changes order of goals via SocketIO.
   *
   * @param {Number} oldIdx Old index of the goal to be re-ordered.
   * @param {Number} newIdx New index of the goal to be re-ordered.
   */
  _moveGoal(oldIdx, newIdx) {
    console.log("[GoalManager] _moveGoal: oldIdx:", oldIdx, "newIdx:", newIdx);
    let socketCb = (success, msg) => {
      console.log("[GoalManager] _moveGoal: success:", success, "msg:", msg);
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

  /** Mission server callback on remote goal addition.
   *
   * @param {Object} goal
   */
  _newGoalCb(goal) {
    console.log("[GoalManager] _newGoalCb: goal:", goal);
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
    console.log("[GoalManager] _removeGoalCb: goal:", goal);
    this.setState((state) => {
      for (var i = 0; i < state.goals.length; i++) {
        if (state.goals[i].id === goal.id) {
          state.goals.splice(i, 1);
          return { goals: state.goals };
        }
      }
      // \todo Update the goal ordering and trigger a refresh. Do we need it?
      // \todo de-select the removed goal if it is selected.
    });
  }

  /** Mission server callback on remote goal start.
   *
   * @param {Object} goal
   */
  _startedGoalCb(goal) {
    console.log("[GoalManager] _startedGoalCb: goal:", goal);
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
    console.log("[GoalManager] _statusCb: data:", data);

    this.setState((state) => {
      state.goals.sort(
        (a, b) => data.queue.indexOf(a.id) - data.queue.indexOf(b.id)
      );
      return { status: data.state, goals: state.goals };
    });
  }

  /** Asks the server to un-pause via SocketIO. */
  _handlePlay() {
    console.log("[GoalManager] _handlePlay");
    let socketCb = (success, msg) => {
      console.log("[GoalManager] _handlePlay: success:", success, "msg:", msg);
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
    console.log("[GoalManager] _handlePause");
    let socketCb = (success, msg) => {
      console.log("[GoalManager] _handlePause: success:", success, "msg:", msg);
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
    console.log("[GoalManager] _handleClear");
    let socketGoalsCb = (success, msg) => {
      console.log("[GoalManager] _handleClear: goals success:", success);
      if (!success) console.error(msg);
      this.setState({ lockGoals: false });
    };
    let socketStatusCb = (success, msg) => {
      console.log("[GoalManager] _handleClear: status success:", success);
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

  _updateWindowHeight() {
    this.setState({ windowHeight: window.innerHeight });
  }
}

export default withStyles(styles)(GoalManager);
