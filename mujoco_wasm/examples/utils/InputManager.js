export class InputManager {
  constructor(options = {}) {
    const {
      allowedKeys = [],
      allowWithinSelectors = ['.dg'],
    } = options;

    this.allowedKeys = new Set(allowedKeys);
    this.allowWithinSelectors = Array.isArray(allowWithinSelectors) ? allowWithinSelectors : [];
    this.enabled = true;
    this.pressed = new Set();

    this._downHandler = this._onKeyDown.bind(this);
    this._upHandler = this._onKeyUp.bind(this);
    this._blurHandler = this.reset.bind(this);

    window.addEventListener('keydown', this._downHandler, { capture: true });
    window.addEventListener('keyup', this._upHandler, { capture: true });
    window.addEventListener('blur', this._blurHandler);
  }

  setAllowedKeys(keys) {
    this.allowedKeys = new Set(keys);
    this.reset();
  }

  setEnabled(enabled) {
    const value = !!enabled;
    if (this.enabled === value) {
      return;
    }
    this.enabled = value;
    if (!this.enabled) {
      this.reset();
    }
  }

  setAllowWithinSelectors(selectors) {
    this.allowWithinSelectors = Array.isArray(selectors) ? selectors : [];
  }

  isPressed(code) {
    return this.pressed.has(code);
  }

  reset() {
    this.pressed.clear();
  }

  destroy() {
    window.removeEventListener('keydown', this._downHandler, true);
    window.removeEventListener('keyup', this._upHandler, true);
    window.removeEventListener('blur', this._blurHandler);
    this.reset();
  }

  _onKeyDown(event) {
    if (!this._shouldHandle(event)) {
      return;
    }
    this.pressed.add(event.code);
    event.preventDefault();
  }

  _onKeyUp(event) {
    if (!this._shouldHandle(event)) {
      return;
    }
    this.pressed.delete(event.code);
    event.preventDefault();
  }

  _shouldHandle(event) {
    if (!this.enabled) {
      return false;
    }

    if (this.allowedKeys.size > 0 && !this.allowedKeys.has(event.code)) {
      return false;
    }

    const target = event.target;
    if (!target) {
      return true;
    }

    const tag = target.tagName ? target.tagName.toUpperCase() : '';
    const isTextInput = tag === 'INPUT' || tag === 'TEXTAREA' || tag === 'SELECT' || target.isContentEditable;

    if (!isTextInput) {
      return true;
    }

    if (this.allowWithinSelectors.length === 0) {
      return false;
    }

    for (let i = 0; i < this.allowWithinSelectors.length; i++) {
      const selector = this.allowWithinSelectors[i];
      if (typeof target.closest === 'function' && target.closest(selector)) {
        return true;
      }
    }

    return false;
  }
}

export default InputManager;
