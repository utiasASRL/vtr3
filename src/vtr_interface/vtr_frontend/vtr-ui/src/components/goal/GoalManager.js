import React from "react";

import shortid from "shortid";

import { withStyles } from "@material-ui/core/styles";
import Button from "@material-ui/core/Button";
import IconButton from "@material-ui/core/IconButton";

import Drawer from "@material-ui/core/Drawer";
import clsx from "clsx";

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
        value={props.value}
        delete={props.delete}
      ></GoalCard>
    </div>
  );
});

const GoalContainer = sortableContainer((props) => {
  const { className } = props;
  return (
    <div className={className} style={{ overflowY: "scroll" }}>
      {props.children}
    </div>
  );
});

// Style
const styles = (theme) => ({
  drawer: (props) => ({
    flexShrink: 100,
  }),
  drawer_paper: {
    backgroundColor: "rgba(255, 255, 255, 0.4)",
  },
  goal_container: (props) => ({
    marginTop: Object.keys(props.currGoal).length !== 0 ? 155 : 20,
    maxHeight: Object.keys(props.currGoal).length !== 0 ? "55vh" : "75vh",
    zIndex: 2000, // \todo This is a magic number.
  }),
  goal_container_helper: {
    zIndex: 2000, // \todo This is a magic number.
  },
  goal_button: {
    marginLeft: "auto",
    marginRight: "auto",
  },
});

class GoalManager extends React.Component {
  constructor(props) {
    super(props);

    this.state = {
      adding_goal: false,
      lock_goals: false,
      goals: [],
    };
  }

  componentDidMount() {
    console.debug("[GoalManager] componentDidMount: Goal manager mounted.");
  }

  render() {
    const { classes, className } = this.props;
    return (
      <Drawer
        className={clsx(classes.drawer, className)}
        variant="persistent"
        anchor="left"
        open={this.props.open}
        classes={{
          paper: clsx(classes.drawer_paper, className),
        }}
      >
        <div style={{ marginLeft: "auto", marginRight: "auto" }}>
          {this.props.currGoalState ? (
            <Button onClick={this._pauseGoal.bind(this)}>Pause</Button>
          ) : (
            <Button onClick={this._startGoal.bind(this)}>Start</Button>
          )}
          <Button onClick={this._clearGoals.bind(this)}>Stop</Button>
        </div>
        <GoalContainer
          onSortEnd={(e) => {
            this._moveGoal(e.oldIndex, e.newIndex);
          }}
          className={classes.goal_container}
          helperClass={classes.goal_container_helper}
          distance={2}
          lockAxis="y"
          useDragHandle
        >
          {this.state.goals.map((value, index) => (
            <Goal
              key={shortid.generate()}
              index={index}
              value={value}
              id={index}
              delete={this._deleteGoal.bind(this)}
              disabled={this.state.lock_goals}
            />
          ))}
        </GoalContainer>
        {this.state.adding_goal && (
          <GoalForm submit={this._submitGoal.bind(this)}></GoalForm>
        )}
        <IconButton
          className={classes.goal_button}
          color="inherit"
          aria-label="add goal"
          edge="start"
          onClick={this._promptGoalForm.bind(this)}
        >
          {this.state.adding_goal ? "Cancel" : "Add Goal"}
        </IconButton>
      </Drawer>
    );
  }

  /** Shows/hides the goal addition form */
  _promptGoalForm() {
    this.setState((state) => ({
      adding_goal: !state.adding_goal,
    }));
  }

  _submitGoal() {
    this.setState((state) => ({
      goals: [...state.goals, { type: "Goal" + String(state.goals.length) }],
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

  _moveGoal(old_id, new_id) {
    this.setState((state) => ({
      goals: arrayMove(state.goals, old_id, new_id),
    }));
  }

  /** Starts the first goal if there exists one. */
  _startGoal() {
    let curr_goal = {};
    this.setState(
      (state, props) => {
        if (Object.keys(props.currGoal).length !== 0) {
          curr_goal = props.currGoal;
          return;
        }
        if (state.goals.length === 0) return;
        curr_goal = state.goals.shift();
        return { goals: state.goals };
      },
      () => this.props.setCurrGoal(curr_goal, true)
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
}

export default withStyles(styles)(GoalManager);
