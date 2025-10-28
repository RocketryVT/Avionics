// @ts-check
import { defineConfig } from 'astro/config';
import starlight from '@astrojs/starlight';

import tailwindcss from '@tailwindcss/vite';

// https://astro.build/config
export default defineConfig({
  site: 'https://rocketryvt.github.io/RocketryVT/Avionics',
  base: '/Avionics',
  integrations: [
      starlight({
          title: 'Rocketry@VT Avionics Documentation',
          social: [{ icon: 'github', label: 'GitHub', href: 'https://github.com/RocketryVT/Avionics' }],
          sidebar: [
              {
                  label: 'Guides',
                  items: [
                      // Each item here is one entry in the navigation menu.
                      { label: 'Example Guide', slug: 'guides/example' },
                  ],
              },
              {
                  label: 'Reference',
                  autogenerate: { directory: 'reference' },
              },
          ],
      }),
	],

  vite: {
    plugins: [tailwindcss()],
  },
});