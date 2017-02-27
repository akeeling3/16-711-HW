import sys
import pygame
from pygame.locals import *
import pymunk
import pymunk.pygame_util

def main():
  pygame.init()
  screen = pygame.display.set_mode((1000, 750))
  pygame.display.set_caption("KDC HW2 Part 4")
  clock = pygame.time.Clock()

  space = pymunk.Space()
  space.gravity = (0.0, -900.0)

  # add floor
  body = pymunk.Body(body_type = pymunk.Body.STATIC)
  body.position = (0, 100)
  floor = pymunk.Segment(body, (-1000, 100), (1000, 100), 1)
  floor.filter = pymunk.ShapeFilter(mask=pymunk.ShapeFilter.ALL_MASKS ^ 0x1)
  floor.friction = 1
  space.add(floor)

  # add cart
  cart_mass = 1
  cart_size = (200, 40)
  cart_moment = pymunk.moment_for_box(cart_mass, cart_size)
  cart_body = pymunk.Body(cart_mass, cart_moment)
  cart_body.position = (300, 300)
  cart_shape = pymunk.Poly.create_box(cart_body, cart_size)
  cart_shape.color = pygame.color.THECOLORS["red"]
  cart_shape.filter = pymunk.ShapeFilter(group=1)

  #add wheels
  wheel_mass = 0.5
  wheel_size = 20
  wheel_moment = pymunk.moment_for_circle(wheel_mass, 0, wheel_size)
  front_wheel_body = pymunk.Body(wheel_mass, wheel_moment)
  back_wheel_body = pymunk.Body(wheel_mass, wheel_moment)
  front_wheel_body.position = (375, 260)
  back_wheel_body.position = (225, 260)
  front_wheel_shape = pymunk.Circle(front_wheel_body, wheel_size)
  back_wheel_shape = pymunk.Circle(back_wheel_body, wheel_size)
  front_wheel_shape.friction = 1
  back_wheel_shape.friction = 1
  front_wheel_shape.color = pygame.color.THECOLORS["black"]
  back_wheel_shape.color = pygame.color.THECOLORS["black"]
  front_wheel_shape.filter = pymunk.ShapeFilter(group=1)
  back_wheel_shape.filter = pymunk.ShapeFilter(group=1)

  #add pole
  pole_mass = 0.1
  pole_size = (1, 200)
  pole_moment = pymunk.moment_for_box(pole_mass, pole_size)
  pole_body = pymunk.Body(pole_mass, pole_moment)
  pole_body.position = (300, 420)
  pole_shape = pymunk.Poly.create_box(pole_body, pole_size)
  pole_shape.color = pygame.color.THECOLORS["black"]
  pole_shape.filter = pymunk.ShapeFilter(categories=0x1, group=1)

  #create joints
  front_joint = pymunk.constraint.PivotJoint(front_wheel_body, cart_body, (375, 260))
  back_joint = pymunk.constraint.PivotJoint(back_wheel_body, cart_body, (225, 260))
  pole_joint = pymunk.constraint.PivotJoint(pole_body, cart_body, (300, 320))

  #add everything to the world
  space.add(cart_body, cart_shape, front_wheel_body, front_wheel_shape,
          back_wheel_body, back_wheel_shape, pole_body, pole_shape,
          front_joint, back_joint, pole_joint)

  draw_options = pymunk.pygame_util.DrawOptions(screen)

  while True:
      for event in pygame.event.get():
          if event.type == QUIT:
              sys.exit(0)
          elif event.type == KEYDOWN and event.key == K_ESCAPE:
              sys.exit(0)

      screen.fill((255,255,255))
      space.debug_draw(draw_options)
      space.step(1/50.0)
      pygame.display.flip()

      # do control here
      p = (pole_body.position.x - cart_body.position.x)
      v = pole_body.velocity_at_local_point((0,0)).x
      theta = pole_body.angle
      dtheta = pole_body.angular_velocity

      a = 1.4142
      b = 0.0199
      c = 2.2621
      d = 0.0722

      f = (a*p)+(b*v)+(c*theta)+(d*dtheta)
      cart_body.apply_impulse_at_local_point((f,0),(0,0))

      clock.tick(50)


if __name__ == '__main__':
    sys.exit(main())
