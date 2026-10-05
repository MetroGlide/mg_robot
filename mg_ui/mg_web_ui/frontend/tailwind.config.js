/** @type {import('tailwindcss').Config} */
const token = (name) => `rgb(var(--${name}) / <alpha-value>)`

export default {
  content: ['./index.html', './src/**/*.{ts,tsx}'],
  darkMode: 'class',
  theme: {
    extend: {
      colors: {
        surface: {
          DEFAULT: token('surface'),
          elevated: token('surface-elevated'),
          sunken: token('surface-sunken'),
        },
        line: token('line'),
        content: token('text'),
        muted: token('muted'),
        accent: token('accent'),
        ok: token('ok'),
        warn: token('warn'),
        error: token('error'),
      },
      boxShadow: {
        card: 'var(--shadow-card)',
      },
    },
  },
  plugins: [],
}
