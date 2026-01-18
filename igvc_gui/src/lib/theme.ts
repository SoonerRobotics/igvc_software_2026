import { writable, derived } from "svelte/store"
import { browser } from "$app/environment"

export type Theme = "light" | "dark" | "system"

const storedTheme = browser ? localStorage.getItem("theme") : null

export const theme = writable<Theme>(
    (storedTheme as Theme) ?? "dark"
)

function getSystemTheme(): "light" | "dark" {
    if (!browser) return "dark"
    return window.matchMedia("(prefers-color-scheme: dark)").matches
        ? "dark"
        : "light"
}

export const resolvedTheme = derived(theme, ($theme) =>
    $theme === "system" ? getSystemTheme() : $theme
)

export function setTheme(value: Theme) {
    theme.set(value)
    if (browser) {
        localStorage.setItem("theme", value)
    }
}

export function toggleTheme() {
    theme.update((t) => {
        const next =
            t === "system"
                ? getSystemTheme() === "dark"
                    ? "light"
                    : "dark"
                : t === "dark"
                    ? "light"
                    : "dark"

        if (browser) {
            localStorage.setItem("theme", next)
        }
        return next
    })
}
