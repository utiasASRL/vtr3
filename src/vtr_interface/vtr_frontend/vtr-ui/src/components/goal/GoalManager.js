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
import GoalForm from "./GoalForm";

const Goal = sortableElement((props) => {
  // \todo check: this div adds smoother animation.
  return (
    <div>
      <GoalCard
        id={props.id}
        goal={props.goal}
        delete={props.delete}
      ></GoalCard>
    </div>
  );
});

const GoalContainer = sortableContainer((props) => {
  const { className, maxHeight } = props;
  return (
    <div
      className={className}
      style={{ overflowY: "scroll", maxHeight: maxHeight }}
    >
      {props.children}
    </div>
  );
});

// Style
const minGap = 5;
const topButtonHeight = 15;
const currGoalCardHeight = 130;
const goalFormHeight = 150;
const styles = (theme) => ({
  drawer: (props) => ({
    flexShrink: 100,
  }),
  drawerPaper: {
    backgroundColor: "rgba(255, 255, 255, 0.2)",
  },
  goalContainer: (props) => ({
    marginTop:
      Object.keys(props.currGoal).length !== 0
        ? topButtonHeight + minGap + currGoalCardHeight
        : topButtonHeight + minGap,
  }),
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
      addingGoal: false,
      lockGoals: false,
      goals: [],
      windowHeight: 0,
    };
  }

  componentDidMount() {
    console.debug("[GoalManager] componentDidMount: Goal manager mounted.");
    window.addEventListener("resize", this._updateWindowHeight.bind(this));
    this._updateWindowHeight();
  }

  componentWillUnmount() {
    window.removeEventListener("resize", this._updateWindowHeight.bind(this));
  }

  render() {
    const {
      classes,
      className,
      addingGoalType,
      setAddingGoalType,
      addingGoalPath,
      setAddingGoalPath,
    } = this.props;
    const { goals, lockGoals, addingGoal } = this.state;
    return (
      <Drawer
        className={clsx(classes.drawer, className)}
        variant="persistent"
        anchor="left"
        open={this.props.open}
        classes={{
          paper: clsx(classes.drawerPaper, className),
        }}
      >
        <div style={{ marginLeft: "auto", marginRight: "auto" }}>
          {this.props.currGoalState ? (
            <Button onClick={this._pauseGoal.bind(this)}>Pause</Button>
          ) : (
            <Button onClick={this._startGoal.bind(this)}>Start</Button>
          )}
          <Button onClick={this._clearGoals.bind(this)}>Clear</Button>
        </div>
        <GoalContainer
          onSortEnd={(e) => {
            this._moveGoal(e.oldIndex, e.newIndex);
          }}
          className={classes.goalContainer}
          helperClass={classes.goalContainerHelper}
          distance={2}
          lockAxis="y"
          useDragHandle
          maxHeight={
            // cannot pass through className because it depends on state.
            this.state.windowHeight -
            (Object.keys(this.props.currGoal).length !== 0
              ? 130 +
                topButtonHeight +
                minGap +
                currGoalCardHeight +
                goalFormHeight
              : 130 + topButtonHeight + minGap + goalFormHeight)
          }
        >
          {goals.map((goal, index) => (
            <Goal
              key={shortid.generate()}
              index={index}
              goal={goal}
              id={index}
              delete={this._deleteGoal.bind(this)}
              disabled={lockGoals}
            />
          ))}
        </GoalContainer>
        {addingGoal && (
          <GoalForm
            goalType={addingGoalType}
            setGoalType={setAddingGoalType}
            goalPath={addingGoalPath}
            setGoalPath={setAddingGoalPath}
            submit={this._submitGoal.bind(this)}
          ></GoalForm>
        )}
        <IconButton
          className={classes.goalButton}
          color="inherit"
          aria-label="add goal"
          edge="start"
          onClick={this._promptGoalForm.bind(this)}
        >
          {addingGoal ? "Cancel" : "Add Goal"}
        </IconButton>
      </Drawer>
    );
  }

  /** Shows/hides the goal addition form */
  _promptGoalForm() {
    this.setState((state) => ({
      addingGoal: !state.addingGoal,
    }));
  }

  _submitGoal(goal) {
    this.setState((state) => ({
      goals: [...state.goals, goal],
    }));
  }

  _deleteGoal(id) {
    this.setState((state) => {
      state.goals.splice(id, 1);
      return {
        goals: state.goals,
      };
    });
  }

  _moveGoal(oldId, newId) {
    this.setState((state) => ({
      goals: arrayMove(state.goals, oldId, newId),
    }));
  }

  /** Starts the first goal if there exists one. */
  _startGoal() {
    let currGoal = {};
    this.setState(
      (state, props) => {
        if (Object.keys(props.currGoal).length !== 0) {
          currGoal = props.currGoal;
          return;
        }
        if (state.goals.length === 0) return;
        currGoal = state.goals.shift();
        return { goals: state.goals };
      },
      () => this.props.setCurrGoal(currGoal, true)
    );
  }

  /** Pauses the current goal if there exists one */
  _pauseGoal() {
    this.props.setCurrGoalState(false);
  }

  /** Cancels all goals */
  _clearGoals() {
    this.setState(
      {
        goals: [],
      },
      () => this.props.setCurrGoal({}, false)
    );
  }

  _updateWindowHeight() {
    this.setState({ windowHeight: window.innerHeight });
  }
}

export default withStyles(styles)(GoalManager);
