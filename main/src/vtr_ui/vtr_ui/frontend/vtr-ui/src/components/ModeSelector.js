import React from "react";

import Box from "@material-ui/core/Box";
import Button from "@material-ui/core/Button";
import FormControl from "@material-ui/core/FormControl";
import InputLabel from "@material-ui/core/InputLabel";
import MenuItem from "@material-ui/core/MenuItem";
import Select from "@material-ui/core/Select";
import { withStyles } from "@material-ui/core/styles";

const styles = (theme) => ({});

class ModeSelector extends React.Component {
  constructor(props) {
    super(props);

    this.modes = {
      vtr: "Visual Teach & Repeat",
      boat: "Boat Project",
      none: "None",
    };

    this.state = { selectedMode: "vtr" };
  }

  render() {
    const { selectedMode } = this.state;
    const { setMode } = this.props;
    return (
      <Box
        style={{ margin: "auto", marginTop: 100, width: 200 }}
        display={"flex"}
        flexDirection={"column"}
      >
        <FormControl style={{ width: 200 }}>
          <InputLabel>Choose Project</InputLabel>
          <Select
            value={selectedMode}
            onChange={(e) => this.setState({ selectedMode: e.target.value })}
          >
            <MenuItem value={"vtr"}>{this.modes.vtr}</MenuItem>
            <MenuItem value={"boat"}>{this.modes.boat}</MenuItem>
            <MenuItem value={"none"}>{this.modes.none}</MenuItem>
          </Select>
        </FormControl>
        <Box style={{ margin: "auto", marginTop: 20, width: 100 }}>
          <Button
            color={"secondary"}
            disableElevation={true}
            fullWidth={false}
            variant={"contained"}
            onClick={() => setMode(selectedMode)}
          >
            confirm
          </Button>
        </Box>
      </Box>
    );
  }
}

export default withStyles(styles)(ModeSelector);
