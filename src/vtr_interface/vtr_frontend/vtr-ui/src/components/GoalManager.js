import React from "react";

import { withStyles } from "@material-ui/core/styles";

import Drawer from "@material-ui/core/Drawer";

import {
  sortableContainer,
  sortableElement,
  sortableHandle,
  arrayMove,
} from "react-sortable-hoc";

const DragHandle = sortableHandle(() => <span>::</span>);

const SortableItem = sortableElement(({ value }) => (
  <div onClick={() => {}}>
    <DragHandle />
    {value}
  </div>
));

const SortableContainer = sortableContainer(({ children }) => {
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
});

class GoalManager extends React.Component {
  constructor(props) {
    super(props);

    this.state = {
      items: ["Item 1", "Item 2", "Item 3", "Item 4", "Item 5", "Item 6"],
    };
  }

  componentDidMount() {
    console.log("Goal manager mounted.");
  }

  render() {
    const { classes } = this.props;
    return (
      <>
        <Drawer
          className={classes.drawer}
          variant="persistent"
          anchor="left"
          open={this.props.open}
          classes={{
            paper: classes.drawer_paper,
          }}
        >
          <SortableContainer
            onSortEnd={this._onSortEnd.bind(this)}
            helperClass={classes.goal_container}
            distance={2}
            lockAxis="y"
            useDragHandle
          >
            {this.state.items.map((value, index) => (
              <SortableItem
                key={`item-${value}`}
                onClick={(e) => {}}
                index={index}
                value={value}
              />
            ))}
          </SortableContainer>
        </Drawer>
      </>
    );
  }

  _onSortEnd(e) {
    console.log(e);
    this.setState(
      (state) => ({
        items: arrayMove(state.items, e.oldIndex, e.newIndex),
      }),
      () => console.log(this.state)
    );
  }
}

export default withStyles(styles)(GoalManager);
