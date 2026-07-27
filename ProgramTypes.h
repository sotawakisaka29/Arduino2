#pragma once

struct vec
{
  int x;
  int y;
};

enum DIR
{
  NORTH,
  EAST,
  SOUTH,
  WEST
};

enum MODE
{
  COMMAND_INPUT_MODE,
  ROUTE_GENERATION_MODE,
  RUN_MODE,
  FINISHED_MODE
};
