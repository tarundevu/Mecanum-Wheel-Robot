import pygame

pygame.init()

pygame.display.set_caption('Quick Start')
window_surface = pygame.display.set_mode((400, 300))

# background
background = pygame.Surface((800, 600))
background.fill(pygame.Color('#808080'))

# buttons and text
pygame.draw.rect(background,(255,255,255),[20,20,80,40])

# data
num = -83.33
enc1 = pygame.font.SysFont(None,30).render(str(num) , True , (0,0,0))

is_running = True

while is_running:
  window_surface.blit(background, (0, 0))
  window_surface.blit(enc1,(30,40))
  pygame.display.flip()


  for event in pygame.event.get():
      if event.type == pygame.QUIT:
          is_running = False
      # data to be displayed
      
  pygame.display.update()