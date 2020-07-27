import React from "react";

import shortid from "shortid";

import { withStyles } from "@material-ui/core/styles";
import IconButton from "@material-ui/core/IconButton";

import Drawer from "@material-ui/core/Drawer";

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

const GoalContainer = sortableContainer(({ children }) => {
  return <div>{children}</div>;
});

// Style
const styles = (theme) => ({
  drawer: (props) => ({
    width: props.panel_width,
    flexShrink: 100,
  }),
  drawer_paper: (props) => ({
    width: props.panel_width,
    backgroundColor: "rgba(255, 255, 255, 0.4)",
  }),
  goal_container: {
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
    console.log("Goal manager mounted.");
  }

  render() {
    const { classes } = this.props;
    return (
      <Drawer
        className={classes.drawer}
        variant="persistent"
        anchor="left"
        open={this.props.open}
        classes={{
          paper: classes.drawer_paper,
        }}
      >
        <GoalContainer
          onSortEnd={(e) => {
            this._moveGoal(e.oldIndex, e.newIndex);
          }}
          helperClass={classes.goal_container}
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
      goals: [...state.goals, "Goal" + String(state.goals.length)],
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
}

export default withStyles(styles)(GoalManager);
